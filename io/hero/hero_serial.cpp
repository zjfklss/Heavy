#include "hero_serial.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <cerrno>
#include "tools/logger.hpp"

namespace hero
{
namespace
{
float read_float(const uint8_t * bytes)
{
  uint32_t bits = uint32_t(bytes[0]) | uint32_t(bytes[1]) << 8 |
                  uint32_t(bytes[2]) << 16 | uint32_t(bytes[3]) << 24;
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
void write_float(uint8_t * bytes, float value)
{
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  for (size_t index = 0; index < 4; ++index) bytes[index] = (bits >> (8 * index)) & 0xff;
}
}
uint16_t crc16(const uint8_t * data, size_t size)
{
  uint16_t crc = 0xffff;
  for (size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) crc = crc & 1 ? (crc >> 1) ^ 0x8408 : crc >> 1;
  }
  return crc;
}
// 视觉 -> 电控，13 字节：
//   [0]      帧头 0xA5
//   [1..4]   pitch  float32(小端，rad)
//   [5..8]   yaw    float32(小端，rad)
//   [9]      控制位：0 不控制(电控保持当前姿态)，1 跟随，2 跟随并开火
//   [10]     保留
//   [11..12] CRC16(对前 11 字节)，低字节在前
// 角度非有限数时一律按"不控制"发出，避免把 NaN 发给云台。
// 注意：云台是否真的听从这个指令，由电控侧(遥控器的自瞄使能)决定，视觉侧不知道也无法替代。
std::array<uint8_t, 13> encode(const io::Command & command)
{
  std::array<uint8_t, 13> bytes{};
  bytes[0] = 0xa5;
  const bool valid = command.control && std::isfinite(command.pitch) && std::isfinite(command.yaw);
  write_float(bytes.data() + 1, valid ? static_cast<float>(command.pitch) : 0);
  write_float(bytes.data() + 5, valid ? static_cast<float>(command.yaw) : 0);
  bytes[9] = valid ? (command.shoot ? 2 : 1) : 0;
  const auto crc = crc16(bytes.data(), 11);
  bytes[11] = crc & 0xff;
  bytes[12] = crc >> 8;
  return bytes;
}
// 电控 -> 视觉，16 字节(从缓冲区里按帧头 0x5A + CRC16 逐帧解析，坏帧丢一个字节重新同步)：
//   [0]      帧头 0x5A
//   [1]      标志位：bit0 敌方颜色，bit1~3 比赛状态，bit4 前哨站状态，bit5 重置 tracker，bit6 相机状态
//   [2..5]   当前云台 pitch float32
//   [6..9]   当前云台 yaw   float32
//   [10..13] 弹速 float32(m/s)
//   [14..15] CRC16(对前 14 字节)
// 注意：协议里没有"自瞄按键"字段。
std::vector<Feedback> parse(std::vector<uint8_t> & buffer)
{
  std::vector<Feedback> result;
  while (!buffer.empty()) {
    buffer.erase(buffer.begin(), std::find(buffer.begin(), buffer.end(), 0x5a));
    if (buffer.size() < 16) break;
    const auto checksum = crc16(buffer.data(), 14);
    if (buffer[14] != (checksum & 0xff) || buffer[15] != (checksum >> 8)) {
      buffer.erase(buffer.begin());
      continue;
    }
    const auto flags = buffer[1];
    Feedback feedback;
    feedback.enemy_color = flags & 1;
    feedback.game_status = (flags >> 1) & 7;
    feedback.outpost_status = flags & 0x10;
    feedback.reset_tracker = flags & 0x20;
    feedback.camera_status = flags & 0x40;
    feedback.pitch = read_float(buffer.data() + 2);
    feedback.yaw = read_float(buffer.data() + 6);
    feedback.bullet_speed = read_float(buffer.data() + 10);
    if (std::isfinite(feedback.pitch) && std::isfinite(feedback.yaw)) result.push_back(feedback);
    buffer.erase(buffer.begin(), buffer.begin() + 16);
  }
  return result;
}
Eigen::Quaterniond attitude(double yaw, double pitch)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()));
}
FeedbackHistory::FeedbackHistory(double fallback, double alpha) : speed_(fallback), alpha_(alpha) {}
void FeedbackHistory::accept(const Feedback & feedback, Clock::time_point stamp)
{
  uint64_t epoch = history_.empty() ? 1 : history_.back().epoch;
  if (!history_.empty() && (std::chrono::duration<double>(stamp - history_.back().stamp).count() > 0.1 ||
      feedback.enemy_color != history_.back().feedback.enemy_color ||
      (feedback.reset_tracker && !history_.back().feedback.reset_tracker))) ++epoch;
  if (feedback.bullet_speed > 10 && feedback.bullet_speed < 20) {
    speed_ = initialized_ ? alpha_ * feedback.bullet_speed + (1 - alpha_) * speed_ : feedback.bullet_speed;
    initialized_ = true;
  }
  history_.push_back({feedback, stamp, epoch, speed_});
  while (history_.size() > 2048) history_.pop_front();
}
std::optional<Sample> FeedbackHistory::latest() const
{
  if (history_.empty()) return std::nullopt;
  return history_.back();
}
std::optional<Eigen::Quaterniond> FeedbackHistory::imu_at(Clock::time_point stamp, double tolerance) const
{
  if (history_.empty()) return std::nullopt;
  const auto delta = [](auto first, auto second) { return std::chrono::duration<double>(first - second).count(); };
  if (stamp <= history_.front().stamp) {
    if (delta(history_.front().stamp, stamp) > tolerance) return std::nullopt;
    return attitude(history_.front().feedback.yaw, history_.front().feedback.pitch);
  }
  for (size_t index = 1; index < history_.size(); ++index) {
    const auto & before = history_[index - 1];
    const auto & after = history_[index];
    if (after.stamp < stamp) continue;
    if (before.epoch != after.epoch || delta(after.stamp, before.stamp) > tolerance) return std::nullopt;
    const double span = delta(after.stamp, before.stamp);
    const double fraction = span > 0 ? delta(stamp, before.stamp) / span : 1;
    return attitude(before.feedback.yaw, before.feedback.pitch).slerp(
      fraction, attitude(after.feedback.yaw, after.feedback.pitch));
  }
  const auto & last = history_.back();
  if (delta(stamp, last.stamp) > tolerance) return std::nullopt;
  return attitude(last.feedback.yaw, last.feedback.pitch);
}
Serial::Serial(const YAML::Node & config)
: config_(YAML::Clone(config)),
  history_(config["bullet_speed"].as<double>(), config["bullet_speed_alpha"].as<double>()),
  timeout_(config["timeout"].as<double>())
{
  thread_ = std::thread(&Serial::run, this);
}
Serial::~Serial()
{
  stop_ = true;
  if (thread_.joinable()) thread_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  close_port();
}
void Serial::close_port()
{
  if (descriptor_ >= 0) tools::logger()->info("[HeroSerial] Port closed");
  if (descriptor_ >= 0) ::close(descriptor_);
  descriptor_ = -1;
  stats_.connected = false;
}
bool Serial::open_port()
{
  descriptor_ = ::open(config_["device"].as<std::string>().c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (descriptor_ < 0) return false;
  termios settings{};
  if (tcgetattr(descriptor_, &settings) != 0) { close_port(); return false; }
  cfmakeraw(&settings);
  settings.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS | CSIZE);
  settings.c_cflag |= CS8 | CLOCAL | CREAD;
  const auto baud = config_["baud_rate"].as<int>() == 921600 ? B921600 : B115200;
  cfsetispeed(&settings, baud);
  cfsetospeed(&settings, baud);
  if (tcsetattr(descriptor_, TCSANOW, &settings) != 0) { close_port(); return false; }
  tcflush(descriptor_, TCIOFLUSH);
  std::cerr << "Hero serial connected: " << config_["device"].as<std::string>() << '\n';
  tools::logger()->info("[HeroSerial] Connected {} @{}", config_["device"].as<std::string>(), config_["baud_rate"].as<int>());
  stats_.connected = true;
  return true;
}
void Serial::run()
{
  std::vector<uint8_t> buffer;
  auto retry = Clock::time_point::min();
  while (!stop_) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (descriptor_ < 0 && Clock::now() >= retry) {
        open_port();
        buffer.clear();
        retry = Clock::now() + std::chrono::seconds(1);
      }
      if (descriptor_ >= 0) {
        pollfd event{descriptor_, POLLIN, 0};
        const int ready = poll(&event, 1, 0);
        if (ready < 0 || event.revents & (POLLERR | POLLHUP | POLLNVAL)) {
          close_port();
          buffer.clear();
        } else if (event.revents & POLLIN) {
          uint8_t bytes[512];
          const auto count = ::read(descriptor_, bytes, sizeof(bytes));
          if (count > 0) {
            buffer.insert(buffer.end(), bytes, bytes + count);
            for (const auto & feedback : parse(buffer)) {
              const auto stamp = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(config_["timestamp_offset"].as<double>()));
              history_.accept(feedback, stamp);
              ++stats_.rx_packets;
            }
          } else if (count == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
            close_port();
            buffer.clear();
          }
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
std::optional<Sample> Serial::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return history_.latest();
}
std::optional<Eigen::Quaterniond> Serial::imu_at(Clock::time_point stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return history_.imu_at(stamp, timeout_);
}
bool Serial::send(io::Command command, uint64_t epoch)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (descriptor_ < 0) { ++stats_.tx_failures; return false; }
  const auto sample = history_.latest();
  if (!sample || sample->epoch != epoch ||
      std::chrono::duration<double>(Clock::now() - sample->stamp).count() > timeout_) {
    if (command.control) ++stats_.tx_suppressed;
    command = {false, false, 0, 0};
  }
  const auto bytes = encode(command);
  size_t sent = 0;
  const auto deadline = Clock::now() + std::chrono::milliseconds(20);
  while (sent < bytes.size() && Clock::now() < deadline) {
    pollfd event{descriptor_, POLLOUT, 0};
    if (poll(&event, 1, 2) <= 0) continue;
    const auto count = ::write(descriptor_, bytes.data() + sent, bytes.size() - sent);
    if (count > 0) sent += static_cast<size_t>(count);
    else break;
  }
  if (sent != bytes.size()) {
    ++stats_.tx_failures;
    tools::logger()->warn("[HeroSerial] Incomplete TX: {}/{} bytes", sent, bytes.size());
    close_port();
    return false;
  }
  ++stats_.tx_packets;
  if (bytes[9] == 2) {
    ++stats_.tx_fire_packets;
    if (stats_.last_status != 2)
      tools::logger()->info("[HeroSerial] TX control=2 bytes=13 pitch={:.6f} yaw={:.6f}", command.pitch, command.yaw);
  }
  stats_.last_status = bytes[9];
  return true;
}
TransportStats Serial::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}
}
