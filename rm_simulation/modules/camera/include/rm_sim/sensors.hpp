#pragma once
#include "rm_sim/protocol.hpp"
#include "rm_sim/image.hpp"
#include <map>
#include <random>

namespace rm_sim {
struct CameraCalibration {
  std::string id="synthetic-pinhole-v1";
  int width=1280, height=1024;
  double fx=900, fy=900, cx=640, cy=512;
  void validate() const;
};
struct Appearance {
  std::string model_id="custom-features-v1", measured_class="armor";
  // In metres: x along Surface.right, y along Surface.up, z along Surface.normal.
  std::vector<Vec3> points;
};
struct SensorConfig {
  bool enabled=true, surface_occlusion=false;
  std::string output="image"; // image (BGR8) or legacy features
  ImageConfig image;
  std::string id="camera";
  CameraCalibration calibration;
  double frequency=60, latency=0, pixel_sigma=0, dropout=0, false_positive_probability=0;
  uint64_t seed=1;
  // Internal truth-to-appearance assignment; target keys never enter SensorPacket.
  std::map<std::string, Appearance> appearances;
  void validate(double dt) const;
};
struct FeatureObservation {
  std::string model_id, measured_class;
  std::vector<Pixel> pixels;
  // Independent isotropic pixel covariance [xx, xy, yy].
  std::array<double,3> covariance{};
};
struct SensorPacket {
  uint64_t sequence=0;
  Tick capture_tick=0, ready_tick=0;
  double capture_time=0, ready_time=0;
  std::string sensor_id, calibration_id;
  std::vector<FeatureObservation> observations;
  std::shared_ptr<const Image> image;
};
struct TruthFeature {
  std::string target_id;
  int plate_id=0, observation_index=-1;
  std::vector<Pixel> ideal_pixels;
  bool visible=false;
  std::string reason;
};
struct TruthAnnotation {
  uint64_t sequence=0;
  Tick capture_tick=0;
  Pose camera_pose;
  std::vector<TruthFeature> features;
};
struct SensorSample { SensorPacket packet; TruthAnnotation truth; };
class Sensor {
 public:
  Sensor(SensorConfig config, double dt);
  SensorSample capture(const Snapshot& snapshot, const Pose& camera_pose, Tick tick);
  const SensorConfig& config() const { return config_; }
 private:
  SensorConfig config_;
  double dt_;
  uint64_t sequence_=0;
  std::mt19937_64 rng_;
  double uniform();
  double noise();
};
} // namespace rm_sim
