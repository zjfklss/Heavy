#include <cmath>
#include <fmt/core.h>

#include <Eigen/Dense>
#include <cmath>
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
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

void print_yaml(
  const std::vector<double> & R_gimbal2imubody_data, const cv::Mat & R_camera2gimbal,
  const cv::Mat & t_camera2gimbal, const Eigen::Vector3d & camera_ypr, double distance,
  const Eigen::Vector3d & board_ypr, const Eigen::Vector3d & rpy, const std::string & output_path)
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
    "相机同理想情况的偏角: yaw {:.6f} pitch {:.6f} roll {:.6f} degree", camera_ypr[0],
    camera_ypr[1], camera_ypr[2]));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "URDF 相机本体 rpy（同 hero.yaml camera_rpy、ROS launch_params rpy）: {:.6f} {:.6f} {:.6f} rad", rpy[0],
    rpy[1], rpy[2]));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format("标定板到世界坐标系原点的水平距离: {:.2f} m", distance));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "标定板同竖直摆放时的偏角: yaw {:.2f} pitch {:.2f} roll {:.2f} degree", board_ypr[0],
    board_ypr[1], board_ypr[2]));
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

  auto board = calib::load_board_config(config_path);
  if (board.p_yaw_to_pitch_mm.norm() < 1.0 && board.p_pitch_to_barrel_mm.norm() < 1.0) {
    fmt::print(stderr, "轴偏置接近 0，robotworld 平移全为零，方程退化。请改用 calibrate_handeye。\n");
    return 1;
  }
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
    "主路径 robotworld；板 {}；偏置 yaw->pitch=[{:.2f},{:.2f},{:.2f}]mm\n",
    board.describe(), board.p_yaw_to_pitch_mm[0],
    board.p_yaw_to_pitch_mm[1], board.p_yaw_to_pitch_mm[2]);

  std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list, rvecs, tvecs;
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
      calib::show_preview(preview, drawing, board.pattern_size(), centers, ok, "calibrate_robotworld_handeye");
    }
    fmt::print("[{}] {}\n", ok ? "success" : "failure", img_path);
    if (!ok) continue;

    Eigen::Matrix3d R_world2gimbal_eigen = R_gimbal2world.transpose();
    Eigen::Vector3d t_gimbal2world = calib::gimbal_origin_in_world_mm(
      ypr_rad, board.p_yaw_to_pitch_mm, board.p_pitch_to_barrel_mm);
    Eigen::Vector3d t_world2gimbal_eigen = -R_world2gimbal_eigen * t_gimbal2world;

    cv::Mat R_world2gimbal_cv, t_world2gimbal;
    cv::eigen2cv(R_world2gimbal_eigen, R_world2gimbal_cv);
    cv::eigen2cv(t_world2gimbal_eigen, t_world2gimbal);

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

    R_world2gimbal_list.emplace_back(R_world2gimbal_cv);
    t_world2gimbal_list.emplace_back(t_world2gimbal);
    rvecs.emplace_back(rvec);
    tvecs.emplace_back(tvec);
    calib::HandEyeSample sample{img_path, R_gimbal2world, t_gimbal2world, {}, {}, centers};
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

  cv::Mat R_gimbal2camera, t_gimbal2camera, R_world2board, t_world2board;
  cv::calibrateRobotWorldHandEye(
    rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list, R_world2board, t_world2board,
    R_gimbal2camera, t_gimbal2camera);
  {
    // 链路检查用 mm，在换算成米之前做
    cv::Mat R_c2g, R_b2w;
    cv::transpose(R_gimbal2camera, R_c2g);
    cv::transpose(R_world2board, R_b2w);
    cv::Mat t_c2g = -R_c2g * t_gimbal2camera, t_b2w = -R_b2w * t_world2board;
    Eigen::Matrix3d R_c2g_e, R_b2w_e;
    Eigen::Vector3d t_c2g_e, t_b2w_e;
    cv::cv2eigen(R_c2g, R_c2g_e);
    cv::cv2eigen(R_b2w, R_b2w_e);
    cv::cv2eigen(t_c2g, t_c2g_e);
    cv::cv2eigen(t_b2w, t_b2w_e);
    calib::print_chain(
      calib::check_chain(samples, obj_template, R_c2g_e, t_c2g_e, camera_matrix, distort_coeffs, &R_b2w_e, &t_b2w_e));
  }
  t_gimbal2camera /= 1e3;
  t_world2board /= 1e3;

  cv::Mat R_camera2gimbal, t_camera2gimbal, R_board2world, t_board2world;
  cv::transpose(R_gimbal2camera, R_camera2gimbal);
  cv::transpose(R_world2board, R_board2world);
  t_camera2gimbal = -R_camera2gimbal * t_gimbal2camera;
  t_board2world = -R_board2world * t_world2board;

  Eigen::Matrix3d R_camera2gimbal_eigen;
  cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
  Eigen::Vector3d camera_ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;

  auto x = t_board2world.at<double>(0);
  auto y = t_board2world.at<double>(1);
  auto distance = std::sqrt(x * x + y * y);

  Eigen::Matrix3d R_board2world_eigen;
  cv::cv2eigen(R_board2world, R_board2world_eigen);
  Eigen::Vector3d board_ypr = tools::eulers(R_board2world_eigen, 2, 1, 0) * 57.3;

  print_yaml(
    board.R_gimbal2imubody, R_camera2gimbal, t_camera2gimbal, camera_ypr, distance, board_ypr,
    calib::camera_rpy_urdf(R_camera2gimbal_eigen), output_path);
  return 0;
}
