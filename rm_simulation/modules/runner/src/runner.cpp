#include "rm_sim/runner.hpp"
#include <limits>
#include <utility>

namespace rm_sim {
namespace {
ActuatorConfig initial_view(ActuatorConfig config,const Snapshot& world) {
  const auto& target=world.targets.front();
  Vec3 aim=target.position;
  const auto origin=config.base_pose.apply(config.yaw_offset);
  if (target.kind==TargetKind::car || target.kind==TargetKind::outpost) {
    // Use an actual receiving face, including the outpost's elevated armor heights.
    double best=-std::numeric_limits<double>::infinity();
    for (const auto& plate:target.surfaces) {
      double distance=norm(origin-plate.center);
      if(distance<1e-9)continue;
      double facing=dot(plate.normal,(origin-plate.center)*(1/distance));
      if(facing>best){best=facing;aim=plate.center;}
    }
  }
  auto clamp=[&](double yaw,double pitch) {
    config.initial_yaw=config.wrap_yaw ? wrap_angle(yaw):std::clamp(yaw,config.yaw_min,config.yaw_max);
    config.initial_pitch=std::clamp(pitch,config.pitch_min,config.pitch_max);
  };
  auto direction=config.base_pose.rotation.inverse(aim-origin);
  if(norm(direction)<1e-9)return config;
  const auto optical=config.camera_mount.rotation.z;
  clamp(std::atan2(direction.y,direction.x)-std::atan2(optical.y,optical.x),
    std::atan2(direction.z,std::hypot(direction.x,direction.y))-
    std::atan2(optical.z,std::hypot(optical.x,optical.y)));
  auto residual=[&](double yaw,double pitch) {
    auto yaw_pose=compose(config.base_pose,{yaw_rotation(yaw),config.yaw_offset});
    auto pitch_pose=compose(yaw_pose,{pitch_rotation(pitch),config.pitch_offset});
    auto camera=compose(pitch_pose,config.camera_mount);
    auto local=camera.inverse(aim);
    return norm(local)>1e-9 ? unit(local)-Vec3{0,0,1}:Vec3{0,0,-1};
  };
  // Refine against the mounted camera, accounting for joint/mount offsets and joint limits.
  for(int i=0;i<32;++i) {
    double yaw=config.initial_yaw,pitch=config.initial_pitch;
    auto r=residual(yaw,pitch);
    if(norm(r)<1e-10)break;
    constexpr double h=1e-5;
    auto a=(residual(yaw+h,pitch)-residual(yaw-h,pitch))*(.5/h);
    auto b=(residual(yaw,pitch+h)-residual(yaw,pitch-h))*(.5/h);
    double aa=dot(a,a)+1e-9,ab=dot(a,b),bb=dot(b,b)+1e-9,det=aa*bb-ab*ab;
    double dy=(-bb*dot(a,r)+ab*dot(b,r))/det;
    double dp=(ab*dot(a,r)-aa*dot(b,r))/det;
    // At a joint stop, optimize the remaining free axis instead of rejecting a coupled step.
    const bool yaw_blocked=!config.wrap_yaw &&
      ((yaw<=config.yaw_min+1e-12 && dy<0) || (yaw>=config.yaw_max-1e-12 && dy>0));
    if(yaw_blocked){dy=0;dp=-dot(b,r)/bb;}
    if((pitch<=config.pitch_min+1e-12 && dp<0) || (pitch>=config.pitch_max-1e-12 && dp>0)) {
      dp=0;dy=yaw_blocked ? 0:-dot(a,r)/aa;
    }
    bool improved=false;
    for(double step=1;step>=1./128;step*=.5) {
      clamp(yaw+std::clamp(dy,-.5,.5)*step,pitch+std::clamp(dp,-.5,.5)*step);
      if(norm(residual(config.initial_yaw,config.initial_pitch))<norm(r)){improved=true;break;}
    }
    if(!improved){clamp(yaw,pitch);break;}
  }
  return config;
}
}
Runner::Runner(RunnerConfig config,std::vector<TargetConfig> targets,
    std::vector<SceneEvent> events,std::vector<ScheduledShot> shots)
 : config_(std::move(config)),world_(config_.world),sensor_(config_.sensor,config_.world.timestep),
   actuator_(config_.actuator,config_.world.timestep),
   sensor_clock_(config_.sensor.frequency,config_.world.timestep),
   feedback_clock_(config_.actuator.feedback_frequency,config_.world.timestep),
   algorithm_clock_(config_.algorithm_frequency,config_.world.timestep),
   log_clock_(config_.log_frequency,config_.world.timestep),events_(std::move(events)),shots_(std::move(shots)) {
  ensure(!targets.empty(), "Runner needs at least one target");
  for (auto target:targets) world_.add_target(std::move(target));
  if(config_.actuator.align_on_start) {
    config_.actuator=initial_view(config_.actuator,world_.snapshot());
    actuator_=Actuator(config_.actuator,config_.world.timestep);
  }
  for (const auto& appearance:config_.sensor.appearances)
    ensure(std::any_of(targets.begin(),targets.end(),[&](const auto& t){return t.id==appearance.first;}),
      "Appearance refers to an unknown target");
  for (const auto& event:events_) {
    auto target=std::find_if(targets.begin(),targets.end(),[&](const auto& t){return t.id==event.target_id;});
    ensure(target!=targets.end(), "Event refers to an unknown target");
    ensure((event.type=="stop_outpost" && target->kind==TargetKind::outpost) ||
      (event.type=="activate_rune" && (target->kind==TargetKind::small_rune || target->kind==TargetKind::big_rune)),
      "Event type does not match target");
  }
  World shot_validator(config_.world);
  for (const auto& shot:shots_) {
    ensure(shot.shot.id.rfind("actuator-shot-",0)!=0, "Reserved actuator shot id prefix");
    shot_validator.launch(shot.shot);
  }
  std::stable_sort(events_.begin(),events_.end(),[](const auto& a,const auto& b){return a.tick<b.tick;});
  std::stable_sort(shots_.begin(),shots_.end(),[](const auto& a,const auto& b){return a.tick<b.tick;});
}
void Runner::enqueue(ControlRequest request,Tick due_tick) {
  ensure(due_tick>=tick_ && due_tick<=1000000000000ULL, "Cannot enqueue a command in the past/outside clock range");
  ensure(std::isfinite(request.yaw) && std::isfinite(request.pitch) && (!request.fire || !request.request_id.empty()),
    "Invalid position command/fire request");
  ActuatorCommand command{command_sequence_++,tick_,due_tick,std::move(request)};
  commands_.emplace(std::make_pair(due_tick,command.sequence),command);
  if(recording_)log_.commands.push_back(command);
  else ++live_commands_;
}
void Runner::schedule(ControlRequest request,Tick due_tick) {
  ensure(!started_, "Prequeue commands before run; create a new Runner for a new round");
  enqueue(std::move(request),due_tick);
}
void Runner::validate_duration(Tick duration) const {
  ensure(duration>0 && duration<=10000000 && duration*config_.world.timestep<=600,
    "Runner duration must be in (0,600] seconds and at most 10000000 ticks");
  if (config_.sensor.enabled && config_.sensor.output=="image") {
    // Logs stream images, but delayed delivery still owns raw pixels. Bound that working set.
    double retention=quantize_delay(config_.sensor.latency,config_.world.timestep)*config_.world.timestep+
      1/config_.algorithm_frequency+config_.world.timestep;
    double frames=std::ceil(std::min(retention,duration*config_.world.timestep)*config_.sensor.frequency)+2;
    double bytes=frames*config_.sensor.calibration.width*config_.sensor.calibration.height*3.;
    ensure(bytes<=512.*1024*1024,"Queued camera images exceed 512 MiB; reduce resolution, frequency, latency or run duration");
  }
  for (const auto& event:events_) ensure(event.tick<duration, "Scene event must precede run end");
  for (const auto& shot:shots_) ensure(shot.tick<duration, "Scheduled shot must precede run end");
}
const RunLog& Runner::run(Tick duration,Algorithm& algorithm) {
  ensure(!started_, "Runner is single-use; create a new instance for a new round");
  validate_duration(duration);
  started_=true;
  size_t event_index=0,shot_index=0;
  AlgorithmInput inbox;
  for (;tick_<duration;++tick_) {
    // Both world and actuator start this iteration at t_n.
    while (event_index<events_.size() && events_[event_index].tick==tick_) {
      const auto& event=events_[event_index++];
      if (event.type=="stop_outpost") world_.stop_outpost(event.target_id);
      else world_.activate_rune(event.target_id);
    }
    while (!commands_.empty() && commands_.begin()->first.first<=tick_) {
      actuator_.apply(commands_.begin()->second);
      commands_.erase(commands_.begin());
    }
    while (shot_index<shots_.size() && shots_[shot_index].tick==tick_) {
      const auto& shot=shots_[shot_index++];
      world_.launch(shot.shot);
      log_.launches.push_back({0,"scene:"+shot.shot.id,tick_,shot.shot});
    }
    for (const auto& event:actuator_.launches()) {
      world_.launch(event.shot);
      log_.launches.push_back(event);
    }
    if (config_.sensor.enabled && sensor_clock_.take(tick_)) {
      auto sample=sensor_.capture(world_.snapshot(),actuator_.truth().camera_pose,tick_);
      if (sample.packet.image && image_sink) image_sink(sample.packet);
      observations_.push_back(sample.packet);
      sample.packet.image.reset(); // Queues own in-flight pixels; logs keep metadata only.
      log_.observations.push_back(std::move(sample.packet));
      log_.annotations.push_back(std::move(sample.truth));
    }
    if (config_.actuator.feedback_enabled && feedback_clock_.take(tick_)) {
      auto feedback=actuator_.feedback();
      feedback_.push_back(feedback);
      log_.feedback.push_back(feedback);
    }
    while (!observations_.empty() && observations_.front().ready_tick<=tick_) {
      inbox.observations.push_back(std::move(observations_.front())); observations_.pop_front();
    }
    while (!feedback_.empty() && feedback_.front().ready_tick<=tick_) {
      inbox.feedback.push_back(feedback_.front()); feedback_.pop_front();
    }
    if (algorithm_clock_.take(tick_)) {
      inbox.tick=tick_; inbox.time=tick_*config_.world.timestep; inbox.timestep=config_.world.timestep;
      for (auto request:algorithm.update(inbox)) {
        Tick delay=std::max<Tick>(1,quantize_delay(config_.actuator.command_latency,config_.world.timestep));
        enqueue(std::move(request),tick_+delay);
      }
      inbox.observations.clear(); inbox.feedback.clear();
    }
    if (log_clock_.take(tick_)) log_.frames.push_back({world_.snapshot(),actuator_.truth()});
    world_.step(); actuator_.advance();
  }
  log_.frames.push_back({world_.snapshot(),actuator_.truth()});
  log_.hits=world_.hits();
  log_.end_tick=tick_;
  log_.pending_observations=observations_.size()+inbox.observations.size();
  log_.pending_feedback=feedback_.size()+inbox.feedback.size();
  log_.pending_commands=commands_.size();
  log_.pending_launches=actuator_.pending_launches();
  return log_;
}
PixelServo::PixelServo(PixelServoConfig config):config_(std::move(config)) {
  config_.calibration.validate();
  ensure(std::isfinite(config_.gain) && config_.gain>0 && config_.gain<=1 &&
    std::isfinite(config_.fire_tolerance) && config_.fire_tolerance>=0 &&
    std::isfinite(config_.fire_interval) && config_.fire_interval>0 &&
    std::isfinite(config_.max_observation_age) && config_.max_observation_age>0,
    "Invalid pixel servo configuration");
}
std::vector<ControlRequest> PixelServo::update(const AlgorithmInput& input) {
  for (const auto& feedback:input.feedback) feedback_=feedback;
  if (!feedback_ || input.observations.empty()) return {};
  const auto& packet=input.observations.back();
  if (packet.calibration_id!=config_.calibration.id ||
    input.time-packet.capture_time>config_.max_observation_age ||
    input.time-feedback_->capture_time>config_.max_observation_age) return {};
  double best=std::numeric_limits<double>::infinity(),dx=0,dy=0;
  for (const auto& observation:packet.observations) {
    if (observation.pixels.empty()) continue;
    Pixel center;
    for (auto p:observation.pixels) { center.x+=p.x; center.y+=p.y; }
    center.x/=observation.pixels.size(); center.y/=observation.pixels.size();
    double x=(center.x-config_.calibration.cx)/config_.calibration.fx;
    double y=(center.y-config_.calibration.cy)/config_.calibration.fy;
    if (x*x+y*y<best) { best=x*x+y*y; dx=-std::atan(x); dy=-std::atan(y); }
  }
  if (!std::isfinite(best)) return {};
  bool fire=config_.fire && std::hypot(dx,dy)<=config_.fire_tolerance && input.time>=next_fire_time_;
  ControlRequest request{feedback_->yaw+config_.gain*dx,feedback_->pitch+config_.gain*dy,fire,{}};
  if (fire) { request.request_id="servo-"+std::to_string(requests_++); next_fire_time_=input.time+config_.fire_interval; }
  return {request};
}
} // namespace rm_sim
