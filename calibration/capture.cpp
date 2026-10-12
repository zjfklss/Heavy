#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
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
  "{require-board   | 1                        | 1=保存前全分辨率复检圆点 }"
  "{skew-ms         | -1                       | 图像与 0x5A 最大时间差(ms)，-1=读 yaml }";

void write_q(const std::string & q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
}

// 采图期间持续读串口。图像戳是入队时刻，不能等取帧之后再读，否则时间差总是偏大。
class AttitudeLog
{
public:
  explicit AttitudeLog(lob_shot::SerialSender & serial) : serial_(serial)
  {
    thread_ = std::thread([this] {
      while (run_) {
        auto fb = serial_.read_feedback();
        if (fb.ok) {
          std::lock_guard<std::mutex> lock(mu_);
          samples_.push_back(fb);
          while (samples_.size() > 400) samples_.pop_front();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  }

  ~AttitudeLog()
  {
    run_ = false;
    if (thread_.joinable()) thread_.join();
  }

  lob_shot::Feedback nearest(std::chrono::steady_clock::time_point t, bool & aligned, int skew_ms) const
  {
    std::lock_guard<std::mutex> lock(mu_);
    lob_shot::Feedback best;
    auto best_dt = std::chrono::milliseconds(skew_ms + 1);
    for (const auto & s : samples_) {
      const auto dt = (t > s.stamp) ? (t - s.stamp) : (s.stamp - t);
      if (dt < best_dt) {
        best_dt = std::chrono::duration_cast<std::chrono::milliseconds>(dt);
        best = s;
      }
    }
    aligned = best.ok && best_dt <= std::chrono::milliseconds(skew_ms);
    return best;
  }

private:
  lob_shot::SerialSender & serial_;
  mutable std::mutex mu_;
  std::deque<lob_shot::Feedback> samples_;
  std::atomic<bool> run_{true};
  std::thread thread_;
};

// 英雄 URDF：yaw 绕 +Z，pitch 关节轴 (0,-1,0)。关节角为弧度。
Eigen::Quaterniond joint_to_q(double yaw, double pitch)
{
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d(0, -1, 0)));
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder, const std::string & attitude,
  std::string port, int baud, bool require_board, int skew_ms)
{
  auto board = calib::load_board_config(config_path);
  calib::CircleGridDetector detector(board);

  auto yaml = YAML::LoadFile(config_path);
  if (yaml["serial_port"]) port = yaml["serial_port"].as<std::string>();
  if (yaml["device_name"]) port = yaml["device_name"].as<std::string>();
  if (yaml["baud"]) baud = yaml["baud"].as<int>();
  if (yaml["baud_rate"]) baud = yaml["baud_rate"].as<int>();
  if (skew_ms < 0) {
    skew_ms = yaml["attitude_skew_ms"] ? yaml["attitude_skew_ms"].as<int>() : 20;
  }

  io::Camera camera(config_path);
  std::unique_ptr<io::CBoard> cboard;
  std::unique_ptr<lob_shot::SerialSender> serial;
  if (attitude == "serial") {
    serial = std::make_unique<lob_shot::SerialSender>(port, baud);
    if (!serial->open()) {
      tools::logger()->error("open serial {} @{} failed", port, baud);
      return;
    }
    tools::logger()->info(
      "attitude=serial {} @{} (0x5A rad, pitch axis -Y, skew {} ms)", port, baud, skew_ms);
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
  std::vector<cv::Point2f> prev_full;
  bool have_prev = false;
  std::unique_ptr<AttitudeLog> attitudes;
  if (serial) attitudes = std::make_unique<AttitudeLog>(*serial);

  while (true) {
    camera.read(img, timestamp);

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    double yaw_deg = 0, pitch_deg = 0, roll_deg = 0;
    bool aligned = false;
    if (attitudes) {
      auto fb = attitudes->nearest(timestamp, aligned, skew_ms);
      if (fb.ok) {
        q = joint_to_q(fb.yaw, fb.pitch);
        yaw_deg = fb.yaw * 180.0 / M_PI;
        pitch_deg = fb.pitch * 180.0 / M_PI;
      }
    } else {
      aligned = true;
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
      preview,
      fmt::format("board {}  sync {}  n={}", preview_ok ? "OK" : "--", aligned ? "OK" : "--", count),
      {40, 160}, (preview_ok && aligned) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
    cv::drawChessboardCorners(preview, board.pattern_size(), draw_pts, preview_ok);

    cv::imshow("s=save q=quit (small circle board)", preview);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key != 's') continue;

    if (serial && !aligned) {
      tools::logger()->warn("图像与 0x5A 时间差超过 {} ms，未保存", skew_ms);
      continue;
    }

    if (require_board) {
      std::vector<cv::Point2f> full;
      if (!detector.detect(img, full, true)) {
        tools::logger()->warn("全分辨率未检出圆点，未保存");
        continue;
      }
      if (calib::align_symmetric_180(have_prev ? &prev_full : nullptr, full)) {
        tools::logger()->info("圆点序相对上一帧翻转 180°");
      }
      prev_full = full;
      have_prev = true;
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
  int skew_ms = cli.get<int>("skew-ms");

  std::filesystem::create_directories(output_folder);
  capture_loop(config_path, output_folder, attitude, port, baud, require_board, skew_ms);
  tools::logger()->warn("注意四元数输出顺序为 wxyz");
  return 0;
}
