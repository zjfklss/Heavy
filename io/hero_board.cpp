#include "hero_board.hpp"

#include <cmath>

#include <yaml-cpp/yaml.h>

#include "tasks/lob_shot/command.hpp"
#include "tools/logger.hpp"

namespace io
{

HeroBoard::HeroBoard(const std::string & config_path)
: bullet_speed(16.5), mode(Mode::auto_aim), queue_(5000)
{
  auto yaml = YAML::LoadFile(config_path);
  if (yaml["mode"]) mode = parse_mode(yaml["mode"].as<std::string>());
  yaml_mode_ = mode;
  if (yaml["bullet_speed"]) bullet_speed = yaml["bullet_speed"].as<double>();
  if (yaml["deploy"]) deploy_ = yaml["deploy"].as<bool>();

  std::string port = "/dev/ttyUSB0";
  int baud = 921600;
  if (yaml["serial_port"]) port = yaml["serial_port"].as<std::string>();
  if (yaml["device_name"]) port = yaml["device_name"].as<std::string>();
  if (yaml["baud"]) baud = yaml["baud"].as<int>();
  if (yaml["baud_rate"]) baud = yaml["baud_rate"].as<int>();

  serial_ = std::make_unique<lob_shot::SerialSender>(port, baud);
  const bool opened = serial_->open();
  if (!opened) {
    tools::logger()->warn("[HeroBoard] open {} @{} failed; imu stays identity", port, baud);
  } else {
    tools::logger()->info("[HeroBoard] {} @{} mode={}", port, baud, MODES[mode]);
  }

  // 先放两帧，imu_at 的 slerp 和 CBoard 一样要队列里已有前后样本
  const auto t0 = std::chrono::steady_clock::now();
  queue_.push({Eigen::Quaterniond::Identity(), t0});
  queue_.push({Eigen::Quaterniond::Identity(), t0 + std::chrono::milliseconds(1)});
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);

  if (opened) {
    rx_run_ = true;
    rx_ = std::thread([this] { rx_loop(); });
  }
}

HeroBoard::~HeroBoard()
{
  rx_run_ = false;
  if (rx_.joinable()) rx_.join();
}

Eigen::Quaterniond HeroBoard::joint_to_q(double yaw, double pitch)
{
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d(0, -1, 0)));
}

Mode HeroBoard::parse_mode(const std::string & name)
{
  for (size_t i = 0; i < MODES.size(); ++i) {
    if (MODES[i] == name) return static_cast<Mode>(i);
  }
  tools::logger()->warn("[HeroBoard] unknown mode {}, use auto_aim", name);
  return Mode::auto_aim;
}

void HeroBoard::rx_loop()
{
  while (rx_run_) {
    auto fb = serial_->read_feedback();
    if (fb.ok) {
      joint_yaw_ = fb.yaw;
      joint_pitch_ = fb.pitch;
      // 与 jlcv_serial_driver 相同：短焦 0、长焦 1
      const int cam = 1 - fb.camera_bit;
      camera_id_ = cam;
      if (cam == 0 && std::abs(fb.bullet_speed - 12.0) < 0.5) bullet_speed = fb.bullet_speed;
      else if (cam == 1 && std::abs(fb.bullet_speed - 16.8) < 0.5) bullet_speed = fb.bullet_speed;
      if (fb.reset_tracker) reset_pending_ = true;
      if (yaml_mode_ == Mode::auto_aim || yaml_mode_ == Mode::outpost) {
        mode = fb.outpost ? Mode::outpost : Mode::auto_aim;
      }
      const auto stamp = fb.stamp.time_since_epoch().count() == 0
                           ? std::chrono::steady_clock::now()
                           : fb.stamp;
      queue_.push({joint_to_q(fb.yaw, fb.pitch), stamp});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

Eigen::Quaterniond HeroBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  // 与 CBoard::imu_at 相同：取时间戳两侧样本做 slerp
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    if (queue_.empty()) break;
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  std::chrono::duration<double> t_ab = data_behind_.timestamp - data_ahead_.timestamp;
  std::chrono::duration<double> t_ac = timestamp - data_ahead_.timestamp;
  if (t_ab.count() < 1e-6) return q_b;

  const double k = t_ac.count() / t_ab.count();
  return q_a.slerp(k, q_b).normalized();
}

void HeroBoard::send(Command command, bool tongji_pitch) const
{
  if (!serial_ || !serial_->is_open()) return;
  lob_shot::Command cmd;
  cmd.control = command.control;
  cmd.shoot = command.shoot;
  cmd.deploy = deploy_;
  cmd.yaw = command.yaw;
  // 同济 Aimer/Buff：世界系抬头为负；英雄 0xA5：抬头为正
  cmd.pitch = tongji_pitch ? -command.pitch : command.pitch;
  serial_->send(cmd);
}

void HeroBoard::latest_joint(double & yaw, double & pitch) const
{
  yaw = joint_yaw_.load();
  pitch = joint_pitch_.load();
}

bool HeroBoard::take_reset()
{
  return reset_pending_.exchange(false);
}

}  // namespace io
