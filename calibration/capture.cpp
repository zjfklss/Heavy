#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "calibration/calib_common.hpp"
#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/lob_shot/lob_serial.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{config-path c   | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o | assets/img_with_q        | 输出文件夹路径   }"
  "{attitude a      | serial                   | 姿态源: serial | can }"
  "{port            | /dev/ttyUSB0             | serial 时串口 }"
  "{baud            | 921600                   | serial 时波特率 }"
  "{require-board   | 1                        | 1=保存前全分辨率复检圆点 }";

void write_q(const std::string & q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
}

Eigen::Quaterniond ypr_to_q(double yaw, double pitch, double roll = 0)
{
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder, const std::string & attitude,
  std::string port, int baud, bool require_board)
{
  auto board = calib::load_board_config(config_path);
  calib::CircleGridDetector detector(board);

  auto yaml = YAML::LoadFile(config_path);
  if (yaml["serial_port"]) port = yaml["serial_port"].as<std::string>();
  if (yaml["device_name"]) port = yaml["device_name"].as<std::string>();
  if (yaml["baud"]) baud = yaml["baud"].as<int>();
  if (yaml["baud_rate"]) baud = yaml["baud_rate"].as<int>();

  io::Camera camera(config_path);
  std::unique_ptr<io::CBoard> cboard;
  std::unique_ptr<lob_shot::SerialSender> serial;
  if (attitude == "serial") {
    serial = std::make_unique<lob_shot::SerialSender>(port, baud);
    if (!serial->open()) {
      tools::logger()->error("open serial {} @{} failed", port, baud);
      return;
    }
    tools::logger()->info("attitude=serial {} @{} (0x5A yaw/pitch)", port, baud);
  } else {
    cboard = std::make_unique<io::CBoard>(config_path);
    tools::logger()->info("attitude=can (CBoard IMU quaternion)");
  }

  tools::logger()->info(
    "小圆点板 {}x{} @ {:.1f}mm；预览 detect_scale={:.2f}", board.pattern_cols, board.pattern_rows,
    board.center_distance_mm, board.detect_scale);

  std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, board.jpeg_quality};
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  int count = 0;
  std::vector<cv::Point2f> centers_preview;

  while (true) {
    camera.read(img, timestamp);

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    double yaw_deg = 0, pitch_deg = 0, roll_deg = 0;
    if (serial) {
      auto fb = serial->read_feedback();
      if (fb.ok) {
        q = ypr_to_q(fb.yaw, fb.pitch, 0);
        yaw_deg = fb.yaw * 57.3;
        pitch_deg = fb.pitch * 57.3;
      }
    } else {
      q = cboard->imu_at(timestamp);
      Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;
      yaw_deg = zyx[0];
      pitch_deg = zyx[1];
      roll_deg = zyx[2];
    }

    // 预览：降采样检测，避免每帧全分辨率 findCirclesGrid
    bool preview_ok = detector.detect(img, centers_preview, false);

    cv::Mat preview;
    cv::resize(img, preview, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    std::vector<cv::Point2f> draw_pts = centers_preview;
    for (auto & p : draw_pts) {
      p.x *= 0.5f;
      p.y *= 0.5f;
    }
    tools::draw_text(preview, fmt::format("yaw   {:.2f}", yaw_deg), {40, 40}, {0, 0, 255});
    tools::draw_text(preview, fmt::format("pitch {:.2f}", pitch_deg), {40, 80}, {0, 0, 255});
    tools::draw_text(preview, fmt::format("roll  {:.2f}", roll_deg), {40, 120}, {0, 0, 255});
    tools::draw_text(
      preview, fmt::format("board {}  n={}", preview_ok ? "OK" : "--", count), {40, 160},
      preview_ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
    cv::drawChessboardCorners(preview, board.pattern_size(), draw_pts, preview_ok);

    cv::imshow("s=save q=quit (small circle board)", preview);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key != 's') continue;

    if (require_board) {
      std::vector<cv::Point2f> full;
      if (!detector.detect(img, full, true)) {
        tools::logger()->warn("全分辨率未检出圆点，未保存");
        continue;
      }
    }

    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img, jpeg_params);
    write_q(q_path, q);
    tools::logger()->info(
      "[{}] Saved yaw={:.2f} pitch={:.2f} -> {}", count, yaw_deg, pitch_deg, output_folder);
  }
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>("config-path");
  auto output_folder = cli.get<std::string>("output-folder");
  auto attitude = cli.get<std::string>("attitude");
  auto port = cli.get<std::string>("port");
  auto baud = cli.get<int>("baud");
  bool require_board = cli.get<int>("require-board") != 0;

  std::filesystem::create_directories(output_folder);
  capture_loop(config_path, output_folder, attitude, port, baud, require_board);
  tools::logger()->warn("注意四元数输出顺序为 wxyz");
  return 0;
}
