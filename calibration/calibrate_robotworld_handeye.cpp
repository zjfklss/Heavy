#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

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
  "{reproj-max r   | 1.0                      | 单帧平均重投影上限(px) }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

void load(
  const std::string & input_folder, const std::string & config_path, const cv::Mat & camera_matrix,
  const cv::Mat & distort_coeffs, double reproj_max, std::vector<double> & R_gimbal2imubody_data,
  std::vector<cv::Mat> & R_world2gimbal_list, std::vector<cv::Mat> & t_world2gimbal_list,
  std::vector<cv::Mat> & rvecs, std::vector<cv::Mat> & tvecs)
{
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto p_yaw_to_pitch = calib::read_vec3_mm(yaml, "p_yaw_to_pitch_mm");
  auto p_pitch_to_barrel = calib::read_vec3_mm(yaml, "p_pitch_to_barrel_mm");

  cv::Size pattern_size(pattern_cols, pattern_rows);
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_gimbal2imubody(R_gimbal2imubody_data.data());

  fmt::print(
    "标定板 {}x{} @ {:.1f}mm；云台偏置 yaw->pitch=[{:.2f},{:.2f},{:.2f}]mm\n", pattern_cols,
    pattern_rows, center_distance_mm, p_yaw_to_pitch[0], p_yaw_to_pitch[1], p_yaw_to_pitch[2]);

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

    auto drawing = img.clone();
    tools::draw_text(drawing, fmt::format("yaw   {:.2f}", ypr[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("pitch {:.2f}", ypr[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("roll  {:.2f}", ypr[2]), {40, 120}, {0, 0, 255});

    std::vector<cv::Point2f> centers_2d;
    auto success = calib::find_circle_grid(img, pattern_size, centers_2d);
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(0);

    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success) continue;

    // WHY: 英雄云台 yaw/pitch 轴不共点，世界系下云台原点随角运动平移
    Eigen::Matrix3d R_world2gimbal_eigen = R_gimbal2world.transpose();
    Eigen::Vector3d t_gimbal2world =
      calib::gimbal_origin_in_world_mm(ypr_rad, p_yaw_to_pitch, p_pitch_to_barrel);
    Eigen::Vector3d t_world2gimbal_eigen = -R_world2gimbal_eigen * t_gimbal2world;

    cv::Mat R_world2gimbal_cv, t_world2gimbal;
    cv::eigen2cv(R_world2gimbal_eigen, R_world2gimbal_cv);
    cv::eigen2cv(t_world2gimbal_eigen, t_world2gimbal);

    cv::Mat rvec, tvec;
    auto centers_3d_ = calib::centers_3d_vertical(pattern_size, center_distance_mm);
    cv::solvePnP(
      centers_3d_, centers_2d, camera_matrix, distort_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    cv::solvePnPRefineLM(centers_3d_, centers_2d, camera_matrix, distort_coeffs, rvec, tvec);
    auto mean_err =
      calib::mean_reproj_error(centers_3d_, centers_2d, rvec, tvec, camera_matrix, distort_coeffs);
    if (mean_err > reproj_max) {
      fmt::print("[rejected] {} reproj={:.3f}px\n", img_path, mean_err);
      continue;
    }
    fmt::print("[accepted] {} reproj={:.3f}px\n", img_path, mean_err);

    R_world2gimbal_list.emplace_back(R_world2gimbal_cv);
    t_world2gimbal_list.emplace_back(t_world2gimbal);
    rvecs.emplace_back(rvec);
    tvecs.emplace_back(tvec);
  }
  fmt::print("有效样本 {}\n", rvecs.size());
}

void print_yaml(
  const std::vector<double> & R_gimbal2imubody_data, const cv::Mat & R_camera2gimbal,
  const cv::Mat & t_camera2gimbal, const Eigen::Vector3d & camera_ypr, double distance,
  const Eigen::Vector3d & board_ypr, const std::string & output_path)
{
  YAML::Emitter result;
  std::vector<double> R_camera2gimbal_data(
    R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t_camera2gimbal_data(
    t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

  result << YAML::BeginMap;
  result << YAML::Key << "R_gimbal2imubody";
  result << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
  result << YAML::Newline << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: yaw {:.6f} pitch {:.6f} roll {:.6f} degree", camera_ypr[0],
    camera_ypr[1], camera_ypr[2]));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: rpy: {:.6f} {:.6f} {:.6f} rad", camera_ypr[2] * M_PI / 180,
    camera_ypr[1] * M_PI / 180, camera_ypr[0] * M_PI / 180));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format("标定板到世界坐标系原点的水平距离: {:.2f} m", distance));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "标定板同竖直摆放时的偏角: yaw {:.2f} pitch {:.2f} roll {:.2f} degree", board_ypr[0],
    board_ypr[1], board_ypr[2]));
  result << YAML::Key << "R_camera2gimbal";
  result << YAML::Value << YAML::Flow << R_camera2gimbal_data;
  result << YAML::Key << "t_camera2gimbal";
  result << YAML::Value << YAML::Flow << t_camera2gimbal_data;
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
  auto camera_info = cli.get<std::string>("camera-info");
  if (camera_info == "none" || camera_info == "true") camera_info.clear();
  auto output_path = cli.get<std::string>("output");
  auto reproj_max = cli.get<double>("reproj-max");

  cv::Mat camera_matrix, distort_coeffs;
  std::string err;
  if (!calib::load_intrinsics_prefer(camera_info, config_path, camera_matrix, distort_coeffs, &err)) {
    fmt::print(stderr, "读内参失败: {}\n", err);
    return 1;
  }

  std::vector<double> R_gimbal2imubody_data;
  std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list;
  std::vector<cv::Mat> rvecs, tvecs;
  load(
    input_folder, config_path, camera_matrix, distort_coeffs, reproj_max, R_gimbal2imubody_data,
    R_world2gimbal_list, t_world2gimbal_list, rvecs, tvecs);
  if (rvecs.empty()) {
    fmt::print(stderr, "无有效手眼样本\n");
    return 1;
  }

  cv::Mat R_gimbal2camera, t_gimbal2camera;
  cv::Mat R_world2board, t_world2board;
  cv::calibrateRobotWorldHandEye(
    rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list, R_world2board, t_world2board,
    R_gimbal2camera, t_gimbal2camera);
  t_gimbal2camera /= 1e3;
  t_world2board /= 1e3;

  cv::Mat R_camera2gimbal, t_camera2gimbal;
  cv::Mat R_board2world, t_board2world;
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
    R_gimbal2imubody_data, R_camera2gimbal, t_camera2gimbal, camera_ypr, distance, board_ypr,
    output_path);
  return 0;
}
