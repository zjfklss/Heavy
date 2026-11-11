#pragma once
#include "rm_sim/actuators.hpp"
#include "rm_sim/sensors.hpp"
#include <deque>
#include <map>
#include <optional>
#include <functional>

namespace rm_sim {
struct AlgorithmInput {
  Tick tick=0;
  double time=0, timestep=0;
  std::vector<SensorPacket> observations;
  std::vector<ActuatorFeedback> feedback;
};
class Algorithm {
 public:
  virtual ~Algorithm() = default;
  virtual std::string version() const = 0;
  virtual std::vector<ControlRequest> update(const AlgorithmInput& input) = 0;
};
struct RunnerConfig {
  WorldConfig world;
  SensorConfig sensor;
  ActuatorConfig actuator;
  double algorithm_frequency=200, log_frequency=50;
};
struct SceneEvent { Tick tick=0; std::string type, target_id; };
struct ScheduledShot { Tick tick=0; Shot shot; };
struct RunFrame { Snapshot world; ActuatorTruth actuator; };
struct LiveUpdate {
  RunFrame frame;
  std::optional<SensorPacket> camera;
  std::vector<Hit> hits;
  uint64_t shots=0,commands=0,camera_frames=0,dropped_camera_jobs=0,contacts=0,effective_hits=0;
  double wall_time=0;
};
struct RunLog {
  std::vector<RunFrame> frames;
  std::vector<SensorPacket> observations;
  std::vector<TruthAnnotation> annotations;
  std::vector<ActuatorFeedback> feedback;
  std::vector<ActuatorCommand> commands;
  std::vector<LaunchEvent> launches;
  std::vector<Hit> hits;
  Tick end_tick=0;
  size_t pending_observations=0, pending_feedback=0, pending_commands=0, pending_launches=0;
};
class Runner {
 public:
  Runner(RunnerConfig config,std::vector<TargetConfig> targets,
    std::vector<SceneEvent> events={},std::vector<ScheduledShot> shots={});
  // Prequeued commands may execute at tick zero. Algorithm-generated commands cannot.
  void schedule(ControlRequest request,Tick due_tick);
  void validate_duration(Tick duration) const;
  const RunLog& run(Tick duration,Algorithm& algorithm);
  // Wall-clock driven, bounded image/log storage. Runs until stop, independent of scenario duration.
  Tick run_live(Algorithm&,const std::function<bool()>& stop,
    const std::function<void(const LiveUpdate&)>& publish,double time_limit=0);
  const RunLog& log() const { return log_; }
  const RunnerConfig& config() const { return config_; }
  // Called at capture; consume or stream pixels without retaining an entire video in RunLog.
  std::function<void(const SensorPacket&)> image_sink;
 private:
  RunnerConfig config_;
  World world_;
  Sensor sensor_;
  Actuator actuator_;
  PeriodicClock sensor_clock_,feedback_clock_,algorithm_clock_,log_clock_;
  Tick tick_=0;
  uint64_t command_sequence_=0;
  bool started_=false;
  bool recording_=true;
  uint64_t live_commands_=0;
  std::vector<SceneEvent> events_;
  std::vector<ScheduledShot> shots_;
  std::map<std::pair<Tick,uint64_t>,ActuatorCommand> commands_;
  std::deque<SensorPacket> observations_;
  std::deque<ActuatorFeedback> feedback_;
  RunLog log_;
  void enqueue(ControlRequest request,Tick due_tick);
};
// A deliberately small image-space servo for exercising the plumbing, not a ballistic solver.
struct PixelServoConfig {
  CameraCalibration calibration;
  double gain=.65, fire_tolerance=.008, fire_interval=.2, max_observation_age=.1;
  bool fire=true;
};
class PixelServo : public Algorithm {
 public:
  explicit PixelServo(PixelServoConfig config={});
  std::string version() const override { return "pixel-servo/1"; }
  std::vector<ControlRequest> update(const AlgorithmInput& input) override;
 private:
  PixelServoConfig config_;
  std::optional<ActuatorFeedback> feedback_;
  double next_fire_time_=0;
  uint64_t requests_=0;
};
} // namespace rm_sim
