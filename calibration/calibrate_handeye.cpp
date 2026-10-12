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
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

void print_yaml(
  const std::vector<double> & R_gimbal2imubody_data, const cv::Mat & R_camera2gimbal,
  const cv::Mat & t_camera2gimbal, const Eigen::Vector3d & ypr, const std::string & output_path)
{
  YAML::Emitter result;
  std::vector<double> R_data(R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t_data(t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

  result << YAML::BeginMap;
  result << YAML::Key << "R_gimbal2imubody" << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
  result << YAML::Newline << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: yaw {:.2f} pitch {:.2f} roll {:.2f} degree", ypr[0], ypr[1], ypr[2]));
  result << YAML::Key << "R_camera2gimbal" << YAML::Value << YAML::Flow << R_data;
  result << YAML::Key << "t_camera2gimbal" << YAML::Value << YAML::Flow << t_data;
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
  double reproj_max = reproj_cli >= 0 ? reproj_cli : board.reproj_max_px;
  calib::CircleGridDetector detector(board);
  auto obj_template = calib::centers_3d_planar(board);

  cv::Mat camera_matrix, distort_coeffs;
  std::string err;
  if (!calib::load_intrinsics_prefer(camera_info, config_path, camera_matrix, distort_coeffs, &err)) {
    fmt::print(stderr, "读内参失败: {}\n", err);
    return 1;
  }

  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(board.R_gimbal2imubody.data());
  fmt::print(
    "辅路径 handeye；板 {}x{} @ {:.1f}mm；偏置 yaw->pitch=[{:.2f},{:.2f},{:.2f}]mm\n",
    board.pattern_cols, board.pattern_rows, board.center_distance_mm, board.p_yaw_to_pitch_mm[0],
    board.p_yaw_to_pitch_mm[1], board.p_yaw_to_pitch_mm[2]);

  std::vector<cv::Mat> R_gimbal2world_list, t_gimbal2world_list, rvecs, tvecs;

  for (int i = 1; true; i++) {
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto q_path = fmt::format("{}/{}.txt", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;
    Eigen::Quaterniond q = calib::read_q(q_path);

    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    Eigen::Matrix3d R_gimbal2world =
      R_gimbal2imubody.transpose() * R_imubody2imuabs * R_gimbal2imubody;
    Eigen::Vector3d ypr_rad = tools::eulers(R_gimbal2world, 2, 1, 0);
    Eigen::Vector3d ypr = ypr_rad * 57.3;

    auto drawing = img;
    tools::draw_text(drawing, fmt::format("yaw   {:.2f}", ypr[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("pitch {:.2f}", ypr[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("roll  {:.2f}", ypr[2]), {40, 120}, {0, 0, 255});

    std::vector<cv::Point2f> centers;
    bool ok = detector.detect(img, centers, true);
    calib::show_preview(preview, drawing, board.pattern_size(), centers, ok, "calibrate_handeye");
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
  }

  if (rvecs.empty()) {
    fmt::print(stderr, "无有效手眼样本\n");
    return 1;
  }

  cv::Mat R_camera2gimbal, t_camera2gimbal;
  cv::calibrateHandEye(
    R_gimbal2world_list, t_gimbal2world_list, rvecs, tvecs, R_camera2gimbal, t_camera2gimbal);
  t_camera2gimbal /= 1e3;

  Eigen::Matrix3d R_camera2gimbal_eigen;
  cv::cv2eigen(R_camera2gimbal, R_camera2gimbal_eigen);
  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
  Eigen::Vector3d ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;

  print_yaml(board.R_gimbal2imubody, R_camera2gimbal, t_camera2gimbal, ypr, output_path);
  return 0;
}
