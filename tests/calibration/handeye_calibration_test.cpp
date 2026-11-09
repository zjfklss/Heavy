// Synthetic hand-eye regression. A vertical board is fixed in the world; the hero gimbal (yaw/pitch axes offset
// by the board config's p_yaw_to_pitch_mm) carries the camera with configs/hero.yaml's extrinsics. Each pose is
// rendered to N.jpg with the hero attitude quaternion in N.txt, then the real solver must recover the extrinsics.
// Usage: handeye_calibration_test <solver> <board.yaml> <hero.yaml> <work dir>
#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "calibration/calib_common.hpp"

namespace
{
constexpr double ppm = 4;  // texture pixels per millimetre
using Matrix3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

// Same convention as hero::attitude(yaw, pitch): yaw about +Z, pitch positive up.
Eigen::Matrix3d attitude(double yaw, double pitch)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()))
    .toRotationMatrix();
}

// Texture whose pixel (u, v) maps to board point (0, (u/ppm - W/2), -(v/ppm - H/2)) mm, i.e. the
// centers_3d_vertical layout: columns along +Y, rows along -Z, origin at the grid centre.
cv::Mat render_board(const calib::BoardConfig & board, double & width_mm, double & height_mm)
{
  const double s = board.center_distance_mm, margin = board.chessboard() ? s : 2 * s;
  width_mm = (board.pattern_cols + (board.chessboard() ? 1 : -1)) * s + 2 * margin;
  height_mm = (board.pattern_rows + (board.chessboard() ? 1 : -1)) * s + 2 * margin;
  cv::Mat texture(int(height_mm * ppm), int(width_mm * ppm), CV_8UC1, cv::Scalar(255));
  if (board.chessboard()) {
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
  return texture;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5) {
    std::cerr << "usage: handeye_calibration_test <solver> <board.yaml> <hero.yaml> <work dir>\n";
    return 2;
  }
  const std::string solver = argv[1], board_path = argv[2];
  const auto hero = YAML::LoadFile(argv[3]);
  const std::filesystem::path work = argv[4];
  std::filesystem::remove_all(work);
  std::filesystem::create_directories(work / "images");
  const auto board = calib::load_board_config(board_path);

  // Ground truth: the hero's configured camera mount (metres in YAML, millimetres here).
  auto R_values = hero["R_camera2gimbal"].as<std::vector<double>>();
  auto t_values = hero["t_camera2gimbal"].as<std::vector<double>>();
  const Eigen::Matrix3d R_c2g = Matrix3(R_values.data());
  const Eigen::Vector3d t_c2g = Eigen::Vector3d(t_values[0], t_values[1], t_values[2]) * 1e3;
  const auto rpy_truth = hero["camera_rpy"].as<std::vector<double>>();
  const Eigen::Matrix3d K = Matrix3(hero["camera_matrix"].as<std::vector<double>>().data());

  // Rendering has no lens distortion, so the solver gets a matching zero-distortion camera_info.
  {
    std::ofstream info(work / "camera_info.yaml");
    info << fmt::format(
      "camera_matrix: [{}, 0, {}, 0, {}, {}, 0, 0, 1]\ndistort_coeffs: [0, 0, 0, 0, 0]\n", K(0, 0), K(0, 2), K(1, 1),
      K(1, 2));
  }

  // Board: vertical, facing the gimbal, centred on the camera axis at the zero pose.
  const auto origin = [&](double yaw, double pitch) {
    // Same kinematics as calib::gimbal_origin_in_world_mm (its pitch argument is the Euler pitch, up negative).
    return calib::gimbal_origin_in_world_mm(
      {yaw, -pitch, 0}, board.p_yaw_to_pitch_mm, board.p_pitch_to_barrel_mm);
  };
  const double distance = board.chessboard() ? 2500 : 1300;
  const Eigen::Vector3d camera0 = origin(0, 0) + t_c2g;
  const Eigen::Vector3d board_centre = camera0 + distance * R_c2g.col(2);
  double width_mm, height_mm;
  const auto texture = render_board(board, width_mm, height_mm);

  int index = 0;
  for (double yaw_deg : {-5.0, -2.5, 0.0, 2.5, 5.0})
    for (double pitch_deg : {-4.0, -1.5, 1.5, 4.0}) {
      const double yaw = yaw_deg * M_PI / 180, pitch = pitch_deg * M_PI / 180;
      const Eigen::Matrix3d R_g2w = attitude(yaw, pitch);
      const Eigen::Matrix3d R_c2w = R_g2w * R_c2g;
      const Eigen::Vector3d t_c2w = R_g2w * t_c2g + origin(yaw, pitch);
      // Board point P_b = (0, y, z) sits at world board_centre + P_b (board axes = world axes).
      // Camera coordinates: R_c2w^T (board_centre + P_b - t_c2w) = A * [y, z]^T + b.
      const Eigen::Matrix3d R_w2c = R_c2w.transpose();
      Eigen::Matrix3d plane;
      plane.col(0) = R_w2c.col(1);  // board +Y
      plane.col(1) = R_w2c.col(2);  // board +Z
      plane.col(2) = R_w2c * (board_centre - t_c2w);
      // Texture (u, v, 1) -> (y, z, 1)
      const Eigen::Matrix3d uv_to_yz{{1 / ppm, 0, -width_mm / 2}, {0, -1 / ppm, height_mm / 2}, {0, 0, 1}};
      // Render 3x supersampled, then area-average down: a direct warp of the sharp texture aliases its edges
      // and biases corner positions. Pixel centres: x_big = S * x + (S - 1) / 2.
      constexpr double S = 3;
      const Eigen::Matrix3d up{{S, 0, (S - 1) / 2}, {0, S, (S - 1) / 2}, {0, 0, 1}};
      const Eigen::Matrix3d H = up * K * plane * uv_to_yz;
      cv::Mat H_cv;
      cv::eigen2cv(H, H_cv);
      cv::Mat big, image;
      cv::warpPerspective(
        texture, big, H_cv, {int(1440 * S), int(1080 * S)}, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 128);
      cv::resize(big, image, {1440, 1080}, 0, 0, cv::INTER_AREA);
      cv::GaussianBlur(image, image, {3, 3}, 0.6);
      cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
      cv::imwrite((work / "images" / fmt::format("{}.jpg", ++index)).string(), image);
      const Eigen::Quaterniond q(R_g2w);
      std::ofstream(work / "images" / fmt::format("{}.txt", index))
        << fmt::format("{} {} {} {}\n", q.w(), q.x(), q.y(), q.z());
    }

  const auto output = work / "result.yaml";
  const auto command = fmt::format(
    "'{}' --config-path='{}' --camera-info='{}' --output='{}' --preview=none '{}' > '{}' 2>&1", solver, board_path,
    (work / "camera_info.yaml").string(), output.string(), (work / "images").string(),
    (work / "log.txt").string());
  if (std::system(command.c_str()) != 0) {
    std::cerr << "solver failed, see " << (work / "log.txt") << '\n';
    return 1;
  }
  int accepted = 0;
  {
    std::ifstream log(work / "log.txt");
    for (std::string line; std::getline(log, line);) accepted += line.rfind("[accepted]", 0) == 0;
  }
  const auto result = YAML::LoadFile(output.string());
  R_values = result["R_camera2gimbal"].as<std::vector<double>>();
  t_values = result["t_camera2gimbal"].as<std::vector<double>>();
  const auto rpy = result["camera_rpy"].as<std::vector<double>>();
  const Eigen::Matrix3d R = Matrix3(R_values.data());
  const Eigen::Vector3d t = Eigen::Vector3d(t_values[0], t_values[1], t_values[2]) * 1e3;
  const double rotation_error = Eigen::AngleAxisd(R.transpose() * R_c2g).angle() * 180 / M_PI;
  const double translation_error = (t - t_c2g).norm();
  double rpy_error = 0;
  for (int i = 0; i < 3; i++) rpy_error = std::max(rpy_error, std::abs(rpy[i] - rpy_truth[i]));
  fmt::print(
    "{} with {}: accepted {}/{} rotation error {:.4f} deg, translation error {:.2f} mm, camera_rpy error {:.5f} "
    "rad\n",
    std::filesystem::path(solver).filename().string(), board.describe(), accepted, index, rotation_error,
    translation_error, rpy_error);
  const bool ok = accepted == index && rotation_error < 0.05 && translation_error < 2 && rpy_error < 1e-3;
  if (!ok) std::cerr << "recovered extrinsics differ from the rendering mount\n";
  return ok ? 0 : 1;
}
