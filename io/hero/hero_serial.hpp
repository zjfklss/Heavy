#pragma once

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>
#include "io/command.hpp"

namespace hero
{
using Clock = std::chrono::steady_clock;
struct Feedback
{
  uint8_t enemy_color = 1;
  uint8_t game_status = 0;
  bool outpost_status = false;
  bool reset_tracker = false;
  bool camera_status = false;
  float pitch = 0;
  float yaw = 0;
  float bullet_speed = 15.8f;
};
uint16_t crc16(const uint8_t * data, size_t size);
std::array<uint8_t, 13> encode(const io::Command & command);
std::vector<Feedback> parse(std::vector<uint8_t> & buffer);
Eigen::Quaterniond attitude(double yaw, double pitch);
struct Sample
{
  Feedback feedback;
  Clock::time_point stamp;
  uint64_t epoch = 0;
  double bullet_speed = 15.8;
};
struct TransportStats
{
  uint64_t rx_packets = 0;
  uint64_t tx_packets = 0;
  uint64_t tx_fire_packets = 0;
  uint64_t tx_failures = 0;
  uint64_t tx_suppressed = 0;
  int last_status = -1;
  bool connected = false;
};
class FeedbackHistory
{
public:
  FeedbackHistory(double fallback, double alpha);
  void accept(const Feedback & feedback, Clock::time_point stamp);
  std::optional<Sample> latest() const;
  std::optional<Eigen::Quaterniond> imu_at(Clock::time_point stamp, double tolerance) const;
private:
  std::deque<Sample> history_;
  double speed_;
  double alpha_;
  bool initialized_ = false;
};
class Serial
{
public:
  explicit Serial(const YAML::Node & config);
  ~Serial();
  std::optional<Sample> state() const;
  std::optional<Eigen::Quaterniond> imu_at(Clock::time_point stamp) const;
  bool send(io::Command command, uint64_t epoch);
  TransportStats stats() const;
private:
  void run();
  bool open_port();
  void close_port();
  YAML::Node config_;
  mutable std::mutex mutex_;
  FeedbackHistory history_;
  int descriptor_ = -1;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  double timeout_;
  TransportStats stats_;
};
}
