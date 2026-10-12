#include <fmt/core.h>

#include <opencv2/opencv.hpp>

#include "calibration/calib_common.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output o       | none                     | 扁平内参输出路径；none=不写 }"
  "{camera-info i  | configs/camera_info.yaml | ROS camera_info 输出路径 }"
  "{preview p      | auto                     | auto|manual|none }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto output_flat = calib::cli_str(cli, "output");
  auto output_ros = calib::cli_str(cli, "camera-info");
  auto preview = calib::parse_preview(cli.get<std::string>("preview"));

  auto board = calib::load_board_config(config_path);
  calib::CircleGridDetector detector(board);
  auto obj_template = calib::centers_3d_planar(board);

  fmt::print(
    "小圆点板 {}x{} @ {:.1f}mm；preview={}\n", board.pattern_cols, board.pattern_rows,
    board.center_distance_mm,
    preview == calib::PreviewMode::None     ? "none"
    : preview == calib::PreviewMode::Manual ? "manual"
                                              : "auto");

  cv::Size img_size;
  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  std::vector<cv::Point2f> prev_centers;
  bool have_prev = false;

  for (int i = 1; true; i++) {
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;
    img_size = img.size();

    std::vector<cv::Point2f> centers;
    bool ok = detector.detect(img, centers, true);
    if (ok && calib::align_symmetric_180(have_prev ? &prev_centers : nullptr, centers)) {
      fmt::print("[flip180] {}\n", img_path);
    }
    if (ok) {
      prev_centers = centers;
      have_prev = true;
    }
    calib::show_preview(
      preview, img, board.pattern_size(), centers, ok, "calibrate_camera (q skips window focus)");
    fmt::print("[{}] {}\n", ok ? "success" : "failure", img_path);
    if (!ok) continue;

    img_points.emplace_back(std::move(centers));
    obj_points.emplace_back(obj_template);
  }

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
    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(
      obj_points[i], rvecs[i], tvecs[i], camera_matrix, distort_coeffs, reprojected);
    total_points += reprojected.size();
    for (size_t j = 0; j < reprojected.size(); j++)
      error_sum += cv::norm(img_points[i][j] - reprojected[j]);
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
