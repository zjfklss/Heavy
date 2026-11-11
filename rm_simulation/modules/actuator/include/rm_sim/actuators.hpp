#pragma once
#include "rm_sim/protocol.hpp"
#include <deque>
#include <random>
#include <set>

namespace rm_sim {
struct ActuatorConfig {
  bool ideal=true, wrap_yaw=true, inherit_muzzle_velocity=true;
  // Place the camera on a visible plate once at startup; never track world truth during a run.
  bool align_on_start=false;
  Pose base_pose{{}, {0,0,.3}};
  Vec3 yaw_offset{}, pitch_offset{};
  Pose camera_mount{{{0,-1,0},{0,0,-1},{1,0,0}}, {}};
  Pose muzzle_mount{};
  double initial_yaw=0, initial_pitch=0;
  double yaw_min=-pi, yaw_max=pi, pitch_min=-pi/3, pitch_max=pi/3;
  double max_speed=6, max_acceleration=60, response_time=.04;
  double command_latency=0, fire_latency=0, fire_interval=.1;
  double muzzle_speed=23, muzzle_speed_sigma=0, shot_lifetime=3;
  bool feedback_enabled=true;
  double feedback_frequency=200, feedback_latency=0, encoder_sigma=0;
  uint64_t seed=2, feedback_seed=3;
  void validate(double dt) const;
};
// Single-shot semantics. Reusing request_id never fires twice in one run.
struct ControlRequest {
  double yaw=0, pitch=0;
  bool fire=false;
  std::string request_id;
};
struct ActuatorCommand {
  uint64_t sequence=0;
  Tick issued_tick=0, due_tick=0;
  ControlRequest request;
};
struct ActuatorTruth {
  Tick tick=0;
  double yaw=0, pitch=0, yaw_velocity=0, pitch_velocity=0;
  Pose camera_pose, muzzle_pose;
  Vec3 muzzle_velocity;
  Tick next_launch_tick=0;
};
struct ActuatorFeedback {
  uint64_t sequence=0;
  Tick capture_tick=0, ready_tick=0;
  double capture_time=0, ready_time=0;
  double yaw=0, pitch=0, yaw_velocity=0, pitch_velocity=0;
};
struct LaunchEvent {
  uint64_t command_sequence=0;
  std::string request_id;
  Tick tick=0;
  Shot shot;
};
class Actuator {
 public:
  Actuator(ActuatorConfig config, double dt);
  void apply(const ActuatorCommand& command);
  std::vector<LaunchEvent> launches();
  void advance();
  ActuatorTruth truth() const;
  ActuatorFeedback feedback();
  const ActuatorConfig& config() const { return config_; }
  size_t pending_launches() const { return pending_.size(); }
 private:
  ActuatorConfig config_;
  double dt_;
  Tick tick_=0, next_launch_=0;
  double yaw_=0, pitch_=0, yaw_velocity_=0, pitch_velocity_=0, goal_yaw_=0, goal_pitch_=0;
  uint64_t feedback_sequence_=0, shot_sequence_=0;
  std::mt19937_64 speed_rng_, feedback_rng_;
  std::set<uint64_t> seen_commands_;
  std::set<std::string> seen_requests_;
  struct PendingShot { Tick due; uint64_t command; std::string request; };
  std::deque<PendingShot> pending_;
  Pose pitch_pose() const;
};
} // namespace rm_sim
