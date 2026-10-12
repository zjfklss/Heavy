#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "calibration/calib_common.hpp"
#include "tools/img_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output o       | none                     | 扁平内参输出路径；none=不写 }"
  "{camera-info i  | configs/camera_info.yaml | ROS camera_info 输出路径 }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

void load(
  const std::string & input_folder, const std::string & config_path, cv::Size & img_size,
  std::vector<std::vector<cv::Point3f>> & obj_points,
  std::vector<std::vector<cv::Point2f>> & img_points)
{
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  cv::Size pattern_size(pattern_cols, pattern_rows);

  for (int i = 1; true; i++) {
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;

    img_size = img.size();
    std::vector<cv::Point2f> centers_2d;
    auto success = calib::find_circle_grid(img, pattern_size, centers_2d);

    auto drawing = img.clone();
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(0);

    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success) continue;

    img_points.emplace_back(centers_2d);
    obj_points.emplace_back(calib::centers_3d_planar(pattern_size, center_distance_mm));
  }
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
  auto output_flat = cli.get<std::string>("output");
  if (output_flat == "none" || output_flat == "true") output_flat.clear();
  auto output_ros = cli.get<std::string>("camera-info");
  if (output_ros == "none" || output_ros == "true") output_ros.clear();

  cv::Size img_size;
  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  load(input_folder, config_path, img_size, obj_points, img_points);
  if (obj_points.empty()) {
    fmt::print(stderr, "无有效内参样本\n");
    return 1;
  }

  cv::Mat camera_matrix, distort_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  auto criteria =
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
  cv::calibrateCamera(
    obj_points, img_points, img_size, camera_matrix, distort_coeffs, rvecs, tvecs, cv::CALIB_FIX_K3,
    criteria);

  double error_sum = 0;
  size_t total_points = 0;
  for (size_t i = 0; i < obj_points.size(); i++) {
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      obj_points[i], rvecs[i], tvecs[i], camera_matrix, distort_coeffs, reprojected_points);
    total_points += reprojected_points.size();
    for (size_t j = 0; j < reprojected_points.size(); j++)
      error_sum += cv::norm(img_points[i][j] - reprojected_points[j]);
  }
  auto error = error_sum / total_points;

  auto flat = calib::emit_camera_flat(camera_matrix, distort_coeffs, error);
  auto ros = calib::emit_camera_info_ros(
    img_size.width, img_size.height, camera_matrix, distort_coeffs, error);
  fmt::print("\n# 扁平（可贴回 calibration.yaml / 任务 yaml）\n{}\n", flat);
  fmt::print("# ROS camera_info\n{}\n", ros);
  calib::write_text(output_flat, flat);
  calib::write_text(output_ros, ros);
  return 0;
}
