#include <cmath>
#include <map>
#include <fmt/core.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "calibration/calib_common.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{camera-info i  | none                     | ROS camera_info；none=读 config }"
  "{output o       | configs/calibration_result.yaml | 结果输出路径 }"
  "{reproj-max r   | -1                       | 单帧平均重投影上限(px)；<0 用 yaml }"
  "{preview p      | auto                     | auto|manual|none }"
  "{method m       | park                     | park|horaud|daniilidis|andreff|tsai；tsai 在小转角时会失败 }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

void print_yaml(
  const std::vector<double> & R_gimbal2imubody_data, const cv::Mat & R_camera2gimbal,
  const cv::Mat & t_camera2gimbal, const Eigen::Vector3d & ypr, const Eigen::Vector3d & rpy,
  const std::string & output_path)
{
  YAML::Emitter result;
  std::vector<double> R_data(R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t_data(t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

  result << YAML::BeginMap;
  result << YAML::Key << "R_gimbal2imubody" << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
  result << YAML::Newline << YAML::Newline;
  result << YAML::Comment(
    "OpenCV 相机系到云台，直接给 solver。不要再乘 URDF 光学系 -pi/2 0 -pi/2");
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: yaw {:.2f} pitch {:.2f} roll {:.2f} degree", ypr[0], ypr[1], ypr[2]));
  result << YAML::Key << "R_camera2gimbal" << YAML::Value << YAML::Flow << R_data;
  result << YAML::Key << "t_camera2gimbal" << YAML::Value << YAML::Flow << t_data;
  // configs/hero.yaml 的 camera_rpy：[roll, pitch, yaw]（rad），与 R_camera2gimbal 一起替换
  result << YAML::Key << "camera_rpy" << YAML::Value << YAML::Flow
         << std::vector<double>{rpy[0], rpy[1], rpy[2]};
  result << YAML::Newline << YAML::EndMap;

  std::string text = result.c_str();
  fmt::print("\n{}\n", text);
  calib::write_text(output_path, text);
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto camera_info = calib::cli_str(cli, "camera-info");
  auto output_path = calib::cli_str(cli, "output");
  if (output_path.empty()) output_path = "configs/calibration_result.yaml";
  auto preview = calib::parse_preview(cli.get<std::string>("preview"));
  double reproj_cli = cli.get<double>("reproj-max");
  const auto method_name = cli.get<std::string>("method");
  const std::map<std::string, cv::HandEyeCalibrationMethod> methods{
    {"tsai", cv::CALIB_HAND_EYE_TSAI}, {"park", cv::CALIB_HAND_EYE_PARK}, {"horaud", cv::CALIB_HAND_EYE_HORAUD},
    {"andreff", cv::CALIB_HAND_EYE_ANDREFF}, {"daniilidis", cv::CALIB_HAND_EYE_DANIILIDIS}};
  if (!methods.count(method_name)) {
    fmt::print(stderr, "未知 --method={}\n", method_name);
    return 2;
  }

  auto board = calib::load_board_config(config_path);
  double reproj_max = reproj_cli >= 0 ? reproj_cli : board.reproj_max_px;
  auto obj_template = calib::centers_3d_vertical(board);

  cv::Mat camera_matrix, distort_coeffs;
  std::string err;
  if (!calib::load_intrinsics_prefer(camera_info, config_path, camera_matrix, distort_coeffs, &err)) {
    fmt::print(stderr, "读内参失败: {}\n", err);
    return 1;
  }

  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(board.R_gimbal2imubody.data());
  fmt::print(
    "辅路径 handeye；板 {}；偏置 yaw->pitch=[{:.2f},{:.2f},{:.2f}]mm\n",
    board.describe(), board.p_yaw_to_pitch_mm[0],
    board.p_yaw_to_pitch_mm[1], board.p_yaw_to_pitch_mm[2]);

  std::vector<cv::Mat> R_gimbal2world_list, t_gimbal2world_list, rvecs, tvecs;
  std::vector<calib::HandEyeSample> samples;
  std::vector<cv::Point2f> prev_centers;
  bool have_prev = false;

  for (auto & detection : calib::detect_folder(input_folder, board)) {
    const auto & img_path = detection.path;
    auto q_path = fmt::format("{}/{}.txt", input_folder, detection.index);
    Eigen::Quaterniond q;
    if (!calib::read_q(q_path, q)) {
      fmt::print("[rejected] {} 缺少或无法解析姿态 {}\n", img_path, q_path);
      continue;
    }

    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    Eigen::Matrix3d R_gimbal2world =
      R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
    Eigen::Vector3d ypr_rad = tools::eulers(R_gimbal2world, 2, 1, 0);
    Eigen::Vector3d joint = calib::joint_from_gimbal_euler(ypr_rad) * 57.3;
    if (std::abs(ypr_rad[2]) > 2.0 * M_PI / 180.0) {
      fmt::print("[rejected] {} roll={:.2f}°，姿态不是纯 yaw/pitch\n", img_path, joint[2]);
      continue;
    }

    auto & centers = detection.points;
    const bool ok = detection.ok;
    if (ok && calib::align_symmetric_180(have_prev ? &prev_centers : nullptr, centers)) {
      fmt::print("[flip180] {}\n", img_path);
    }
    if (ok) {
      prev_centers = centers;
      have_prev = true;
    }
    if (preview != calib::PreviewMode::None) {
      auto drawing = cv::imread(img_path);
      tools::draw_text(drawing, fmt::format("joint yaw   {:.2f}", joint[0]), {40, 40}, {0, 0, 255});
      tools::draw_text(drawing, fmt::format("joint pitch {:.2f}", joint[1]), {40, 80}, {0, 0, 255});
      tools::draw_text(drawing, fmt::format("roll        {:.2f}", joint[2]), {40, 120}, {0, 0, 255});
      calib::show_preview(preview, drawing, board.pattern_size(), centers, ok, "calibrate_handeye");
    }
    fmt::print("[{}] {}\n", ok ? "success" : "failure", img_path);
    if (!ok) continue;

    Eigen::Vector3d t_g2w = calib::gimbal_origin_in_world_mm(
      ypr_rad, board.p_yaw_to_pitch_mm, board.p_pitch_to_barrel_mm);
    cv::Mat t_gimbal2world = (cv::Mat_<double>(3, 1) << t_g2w[0], t_g2w[1], t_g2w[2]);
    cv::Mat R_gimbal2world_cv;
    cv::eigen2cv(R_gimbal2world, R_gimbal2world_cv);

    cv::Mat rvec, tvec;
    cv::solvePnP(
      obj_template, centers, camera_matrix, distort_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    cv::solvePnPRefineLM(obj_template, centers, camera_matrix, distort_coeffs, rvec, tvec);
    auto mean_err =
      calib::mean_reproj_error(obj_template, centers, rvec, tvec, camera_matrix, distort_coeffs);
    if (mean_err > reproj_max) {
      fmt::print("[rejected] {} reproj={:.3f}px\n", img_path, mean_err);
      continue;
    }
    fmt::print("[accepted] {} reproj={:.3f}px\n", img_path, mean_err);

    R_gimbal2world_list.emplace_back(R_gimbal2world_cv);
    t_gimbal2world_list.emplace_back(t_gimbal2world);
    rvecs.emplace_back(rvec);
    tvecs.emplace_back(tvec);
    calib::HandEyeSample sample{img_path, R_gimbal2world, t_g2w, {}, {}, centers};
    cv::Mat R_b2c;
    cv::Rodrigues(rvec, R_b2c);
    cv::cv2eigen(R_b2c, sample.R_board2camera);
    cv::cv2eigen(tvec, sample.t_board2camera_mm);
    samples.push_back(std::move(sample));
  }
  fmt::print("有效样本 {}\n", rvecs.size());

  if (rvecs.size() < 3) {
    fmt::print(stderr, "有效手眼样本少于 3 组，无法求解\n");
    return 1;
  }

  cv::Mat R_camera2gimbal, t_camera2gimbal;
  cv::calibrateHandEye(
    R_gimbal2world_list, t_gimbal2world_list, rvecs, tvecs, R_camera2gimbal, t_camera2gimbal,
    methods.at(method_name));
  Eigen::Matrix3d R_camera2gimbal_eigen;
  Eigen::Vector3d t_camera2gimbal_mm;
  cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
  cv::cv2eigen(t_camera2gimbal, t_camera2gimbal_mm);
  t_camera2gimbal /= 1e3;
  fmt::print("方法 {}\n", method_name);
  const auto check = calib::check_chain(
    samples, obj_template, R_camera2gimbal_eigen, t_camera2gimbal_mm, camera_matrix, distort_coeffs);
  calib::print_chain(check);

  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
  Eigen::Vector3d ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;

  print_yaml(
    board.R_gimbal2imubody, R_camera2gimbal, t_camera2gimbal, ypr, calib::camera_rpy_urdf(R_camera2gimbal_eigen),
    output_path);
  return 0;
}
