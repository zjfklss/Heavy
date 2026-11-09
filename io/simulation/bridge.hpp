#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <deque>
#include <filesystem>
#include <optional>
#include <opencv2/core.hpp>

#include "io/command.hpp"
#include "io/hero/hero_serial.hpp"
#include "rm_sim/adapters/server.hpp"

namespace io::simulation
{
// Hero angles: radians, yaw left positive, pitch up positive (the rm-sim "generic" profile).
inline constexpr const char * adapter = "generic";

// Validated hero YAML with model paths resolved to absolute paths.
YAML::Node load_base_config(const std::string & path);

// World-frame yaw/pitch of the barrel axis for a hero attitude rotated by the simulator base.
struct Angles
{
  double yaw, pitch;
};
Angles to_world(const Eigen::Matrix3d & base, double yaw, double pitch);
Angles to_simulator(const Eigen::Matrix3d & base, double yaw, double pitch);

// This module maps transport, sensor timestamps, calibration and command fields only.
struct Frame
{
  cv::Mat bgr;
  hero::Clock::time_point stamp;
  Eigen::Quaterniond attitude;
  uint64_t sequence;
};
struct Step
{
  std::vector<Frame> frames;
  double time;
  // Diagnostics in the same world convention as the hero command.
  struct Feedback
  {
    double time, yaw, pitch;
  };
  std::vector<Feedback> feedback;
};
class Bridge
{
public:
  explicit Bridge(rm_sim::adapters::ServerOptions options);
  ~Bridge();
  void accept(const std::string & base_config);
  const std::string & config_path() const { return config_path_; }
  double bullet_speed() const { return bullet_speed_; }
  int enemy_color() const { return enemy_color_; }
  bool realtime() const { return realtime_; }
  std::optional<Step> next();
  void reply(const std::optional<io::Command> &, nlohmann::json telemetry);

private:
  rm_sim::adapters::ProtocolServer server_;
  std::filesystem::path temporary_;
  std::string config_path_, session_;
  hero::FeedbackHistory history_{15.8, 1};
  hero::Clock::time_point epoch_ = hero::Clock::time_point{};
  Eigen::Matrix3d base_rotation_ = Eigen::Matrix3d::Identity();
  struct Pending
  {
    cv::Mat bgr;
    double capture;
    uint64_t sequence;
  };
  std::deque<Pending> pending_;
  double latest_feedback_ = -1, time_ = 0, feedback_age_ = .05, wait_limit_ = 1;
  double bullet_speed_ = 15.8, fire_interval_ = .1, next_fire_ = 0;
  int enemy_color_ = 1;
  bool realtime_ = false;
  uint64_t frames_ = 0, feedback_ = 0, dropped_ = 0, shots_ = 0;
  rm_sim::adapters::Command last_command_;
  size_t queued_bytes_ = 0;
  hero::Clock::time_point stamp(double seconds) const;
};
}  // namespace io::simulation
