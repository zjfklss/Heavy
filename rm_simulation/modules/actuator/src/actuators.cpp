#include "rm_sim/actuators.hpp"
#include <utility>

namespace rm_sim {
void ActuatorConfig::validate(double dt) const {
  base_pose.validate(); camera_mount.validate(); muzzle_mount.validate();
  ensure(finite(yaw_offset) && finite(pitch_offset), "Invalid joint offset");
  for (double v:{initial_yaw,initial_pitch,yaw_min,yaw_max,pitch_min,pitch_max,max_speed,
    max_acceleration,response_time,muzzle_speed,muzzle_speed_sigma,shot_lifetime,encoder_sigma})
    ensure(std::isfinite(v), "Non-finite actuator configuration");
  ensure(yaw_min<yaw_max && pitch_min<pitch_max && pitch_min>=-pi/2 && pitch_max<=pi/2 &&
    initial_pitch>=pitch_min && initial_pitch<=pitch_max &&
    (wrap_yaw || (initial_yaw>=yaw_min && initial_yaw<=yaw_max)), "Invalid joint limits/initial angles");
  ensure(max_speed>0 && max_acceleration>0 && response_time>=0 && muzzle_speed>0 &&
    muzzle_speed_sigma>=0 && shot_lifetime>0 && encoder_sigma>=0, "Invalid actuator dynamics");
  ensure(fire_interval>0, "Fire interval must be positive");
  for (double delay:{command_latency,fire_latency,fire_interval,feedback_latency}) quantize_delay(delay,dt);
  PeriodicClock clock(feedback_frequency,dt); (void)clock;
}
Actuator::Actuator(ActuatorConfig config,double dt) : config_(std::move(config)),dt_(dt),
  speed_rng_(config_.seed),feedback_rng_(config_.feedback_seed) {
  config_.validate(dt);
  yaw_=goal_yaw_=config_.wrap_yaw ? wrap_angle(config_.initial_yaw) : config_.initial_yaw;
  pitch_=goal_pitch_=config_.initial_pitch;
}
Pose Actuator::pitch_pose() const {
  auto yaw=compose(config_.base_pose,{yaw_rotation(yaw_),config_.yaw_offset});
  return compose(yaw,{pitch_rotation(pitch_),config_.pitch_offset});
}
ActuatorTruth Actuator::truth() const {
  const auto pitch=pitch_pose();
  auto camera=compose(pitch,config_.camera_mount), muzzle=compose(pitch,config_.muzzle_mount);
  Vec3 yaw_origin=config_.base_pose.apply(config_.yaw_offset);
  auto yaw_frame=compose(config_.base_pose,{yaw_rotation(yaw_),config_.yaw_offset});
  Vec3 yaw_omega=config_.base_pose.rotation.z*yaw_velocity_;
  Vec3 pitch_omega=yaw_frame.rotation.y*(-pitch_velocity_);
  Vec3 velocity=cross(yaw_omega,muzzle.translation-yaw_origin)+
    cross(pitch_omega,muzzle.translation-pitch.translation);
  return {tick_,yaw_,pitch_,yaw_velocity_,pitch_velocity_,camera,muzzle,velocity,next_launch_};
}
void Actuator::apply(const ActuatorCommand& command) {
  ensure(command.due_tick==tick_ && command.issued_tick<=command.due_tick,
    "Command must be applied at its due tick");
  const auto& r=command.request;
  ensure(std::isfinite(r.yaw) && std::isfinite(r.pitch) && (!r.fire || !r.request_id.empty()),
    "Invalid position command or fire request id");
  if (!seen_commands_.insert(command.sequence).second) return;
  goal_yaw_=config_.wrap_yaw ? wrap_angle(r.yaw) : std::clamp(r.yaw,config_.yaw_min,config_.yaw_max);
  goal_pitch_=std::clamp(r.pitch,config_.pitch_min,config_.pitch_max);
  if (config_.ideal) {
    // Ideal position steps have no finite physical derivative; inherited speed is zero.
    yaw_=goal_yaw_; pitch_=goal_pitch_; yaw_velocity_=pitch_velocity_=0;
  }
  if (r.fire && seen_requests_.insert(r.request_id).second) {
    Tick due=std::max(tick_+quantize_delay(config_.fire_latency,dt_),next_launch_);
    pending_.push_back({due,command.sequence,r.request_id});
    next_launch_=due+std::max<Tick>(1,quantize_delay(config_.fire_interval,dt_));
  }
}
std::vector<LaunchEvent> Actuator::launches() {
  std::vector<LaunchEvent> events;
  while (!pending_.empty() && pending_.front().due<=tick_) {
    const auto pending=pending_.front(); pending_.pop_front();
    const auto actual=truth();
    double speed=config_.muzzle_speed;
    if (config_.muzzle_speed_sigma>0)
      speed=std::max(1e-6,speed+std::normal_distribution<double>(0,config_.muzzle_speed_sigma)(speed_rng_));
    Vec3 velocity=actual.muzzle_pose.rotation.x*speed;
    if (config_.inherit_muzzle_velocity) velocity=velocity+actual.muzzle_velocity;
    events.push_back({pending.command,pending.request,tick_,
      {"actuator-shot-"+std::to_string(shot_sequence_++),actual.muzzle_pose.translation,
        velocity,config_.shot_lifetime}});
  }
  return events;
}
void Actuator::advance() {
  if (!config_.ideal) {
    auto axis=[&](double& angle,double& velocity,double target,bool wrap,double minimum,double maximum) {
      double error=wrap ? wrap_angle(target-angle) : target-angle;
      double desired=error/std::max(config_.response_time,dt_);
      double braking=std::sqrt(2*config_.max_acceleration*std::abs(error));
      double limit=std::min(config_.max_speed,braking);
      desired=std::clamp(desired,-limit,limit);
      double next_velocity=velocity+std::clamp(desired-velocity,
        -config_.max_acceleration*dt_,config_.max_acceleration*dt_);
      angle+=(velocity+next_velocity)*dt_/2;
      velocity=next_velocity;
      if (wrap) angle=wrap_angle(angle);
      else if (angle<minimum || angle>maximum) {
        angle=std::clamp(angle,minimum,maximum); velocity=0; // Perfectly inelastic hard stop.
      }
    };
    axis(yaw_,yaw_velocity_,goal_yaw_,config_.wrap_yaw,config_.yaw_min,config_.yaw_max);
    axis(pitch_,pitch_velocity_,goal_pitch_,false,config_.pitch_min,config_.pitch_max);
  }
  ++tick_;
}
ActuatorFeedback Actuator::feedback() {
  auto noise=[&] { return config_.encoder_sigma==0 ? 0 :
    std::normal_distribution<double>(0,config_.encoder_sigma)(feedback_rng_); };
  Tick ready=tick_+quantize_delay(config_.feedback_latency,dt_);
  double yaw=yaw_+noise();
  if (config_.wrap_yaw) yaw=wrap_angle(yaw);
  return {feedback_sequence_++,tick_,ready,tick_*dt_,ready*dt_,yaw,pitch_+noise(),yaw_velocity_,pitch_velocity_};
}
} // namespace rm_sim
