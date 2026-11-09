// Renders synthetic photos of each board with a known camera, runs the real calibrate_camera binary on them,
// and checks the recovered intrinsics. Usage: board_calibration_test <calibrate_camera> <board.yaml> <work dir>
#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "calibration/calib_common.hpp"

namespace
{
constexpr double ppm = 4;  // texture pixels per millimetre

// Board texture plus the texture-pixel -> board-metre mapping centred on the board.
cv::Mat render_board(const calib::BoardConfig & board, cv::Matx33d & texture_to_metres)
{
  const double s = board.center_distance_mm, margin = board.chessboard() ? s : 2 * s;
  const double width = (board.pattern_cols + (board.chessboard() ? 1 : -1)) * s + 2 * margin;
  const double height = (board.pattern_rows + (board.chessboard() ? 1 : -1)) * s + 2 * margin;
  cv::Mat texture(int(height * ppm), int(width * ppm), CV_8UC1, cv::Scalar(255));
  if (board.chessboard()) {
    // pattern_cols x pattern_rows inner corners need (cols + 1) x (rows + 1) squares.
    for (int r = 0; r <= board.pattern_rows; ++r)
      for (int c = 0; c <= board.pattern_cols; ++c)
        if ((r + c) % 2 == 0)
          cv::rectangle(
            texture, cv::Rect(int((margin + c * s) * ppm), int((margin + r * s) * ppm), int(s * ppm), int(s * ppm)),
            cv::Scalar(0), cv::FILLED);
  } else {
    for (int r = 0; r < board.pattern_rows; ++r)
      for (int c = 0; c < board.pattern_cols; ++c)
        cv::circle(
          texture, {int((margin + c * s) * ppm), int((margin + r * s) * ppm)}, int(s / 4 * ppm), cv::Scalar(0),
          cv::FILLED, cv::LINE_AA);
  }
  texture_to_metres = {1 / (ppm * 1000), 0, -width / 2000, 0, 1 / (ppm * 1000), -height / 2000, 0, 0, 1};
  return texture;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "usage: board_calibration_test <calibrate_camera> <board.yaml> <work dir>\n";
    return 2;
  }
  const std::string tool = argv[1], config = argv[2];
  const std::filesystem::path work = argv[3];
  std::filesystem::remove_all(work);
  std::filesystem::create_directories(work / "images");
  const auto board = calib::load_board_config(config);
  const cv::Matx33d K(3546.58523, 0, 730.97855, 0, 3545.36711, 572.93391, 0, 0, 1);
  cv::Matx33d A;
  const auto texture = render_board(board, A);
  // Distance chosen so the board fills a useful part of the 1440x1080 frame.
  const double distance = board.chessboard() ? 2.2 : 1.1;
  int index = 0;
  for (double ax : {-0.35, 0.0, 0.35})
    for (double ay : {-0.35, 0.0, 0.35})
      for (double shift : {-1.0, 1.0}) {
        const cv::Vec3d rvec(ax, ay, 0.1 * shift);
        cv::Matx33d R;
        cv::Rodrigues(rvec, R);
        const cv::Vec3d t(0.06 * shift * distance, -0.04 * shift * distance, distance + 0.15 * shift);
        const cv::Matx33d plane(R(0, 0), R(0, 1), t[0], R(1, 0), R(1, 1), t[1], R(2, 0), R(2, 1), t[2]);
        const cv::Matx33d H = K * plane * A;
        cv::Mat image;
        cv::warpPerspective(texture, image, cv::Mat(H), {1440, 1080}, cv::INTER_AREA, cv::BORDER_CONSTANT, 128);
        cv::GaussianBlur(image, image, {3, 3}, 0.6);
        cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        cv::imwrite((work / "images" / fmt::format("{}.jpg", ++index)).string(), image);
      }
  const auto output = work / "intrinsics.yaml";
  const auto command = fmt::format(
    "'{}' --config-path='{}' --output='{}' --camera-info=none --preview=none '{}' > '{}' 2>&1", tool, config,
    output.string(), (work / "images").string(), (work / "log.txt").string());
  if (std::system(command.c_str()) != 0) {
    std::cerr << "calibrate_camera failed, see " << (work / "log.txt") << '\n';
    return 1;
  }
  const auto result = YAML::LoadFile(output.string())["camera_matrix"].as<std::vector<double>>();
  int detected = 0;
  {
    std::ifstream log(work / "log.txt");
    for (std::string line; std::getline(log, line);) detected += line.rfind("[success]", 0) == 0;
  }
  fmt::print(
    "{}: detected {}/{} fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}\n", board.describe(), detected, index, result[0],
    result[4], result[2], result[5]);
  // Like real sessions, a few steep shots may be rejected; the solve must still be accurate.
  const bool ok = detected >= index * 4 / 5 && std::abs(result[0] / K(0, 0) - 1) < 0.01 &&
                  std::abs(result[4] / K(1, 1) - 1) < 0.01 && std::abs(result[2] - K(0, 2)) < 15 &&
                  std::abs(result[5] - K(1, 2)) < 15;
  if (!ok) std::cerr << "recovered intrinsics differ from the rendering camera\n";
  return ok ? 0 : 1;
}
