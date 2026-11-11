#include "rm_sim/sensors.hpp"
#include <utility>

namespace rm_sim {
void CameraCalibration::validate() const {
  ensure(!id.empty() && width>0 && height>0 && width<=65536 && height<=65536,
    "Invalid camera identity/image size");
  ensure(std::isfinite(fx) && std::isfinite(fy) && fx>0 && fy>0 &&
    std::isfinite(cx) && std::isfinite(cy), "Invalid camera intrinsics");
}
void SensorConfig::validate(double dt) const {
  calibration.validate();
  ensure(output=="image" || output=="features", "Sensor output must be image or features");
  image.validate();
  if (output=="image") {
    ensure(size_t(calibration.width)*calibration.height<=16777216, "Camera image exceeds 16 megapixels");
    ensure(calibration.fx<=1e9 && calibration.fy<=1e9 && std::abs(calibration.cx)<=1e9 && std::abs(calibration.cy)<=1e9,
      "Image intrinsics exceed supported range");
  }
  ensure(!id.empty(), "Sensor id is required");
  PeriodicClock clock(frequency,dt);
  (void)clock;
  quantize_delay(latency,dt);
  ensure(std::isfinite(pixel_sigma) && pixel_sigma>=0 &&
    std::isfinite(dropout) && dropout>=0 && dropout<=1 &&
    std::isfinite(false_positive_probability) && false_positive_probability>=0 &&
    false_positive_probability<=1, "Invalid sensor noise probabilities");
  for (const auto& entry : appearances) {
    const auto& a=entry.second;
    ensure(!entry.first.empty() && !a.model_id.empty() && !a.measured_class.empty() &&
      !a.points.empty() && a.points.size()<=64, "Invalid appearance model");
    for (auto p:a.points) ensure(finite(p), "Non-finite feature point");
  }
}
Sensor::Sensor(SensorConfig config,double dt) : config_(std::move(config)),dt_(dt),rng_(config_.seed) {
  config_.validate(dt);
}
double Sensor::uniform() { return std::generate_canonical<double,53>(rng_); }
double Sensor::noise() {
  return config_.pixel_sigma==0 ? 0 : std::normal_distribution<double>(0,config_.pixel_sigma)(rng_);
}
SensorSample Sensor::capture(const Snapshot& snapshot,const Pose& camera_pose,Tick tick) {
  camera_pose.validate();
  ensure(tick<=1000000000000ULL && std::abs(snapshot.time-tick*dt_)<1e-8,
    "Sensor snapshot and pose must share capture tick");
  SensorSample sample;
  auto& packet=sample.packet;
  packet.sequence=sequence_++;
  packet.capture_tick=tick;
  packet.ready_tick=tick+quantize_delay(config_.latency,dt_);
  packet.capture_time=tick*dt_;
  packet.ready_time=packet.ready_tick*dt_;
  packet.sensor_id=config_.id;
  packet.calibration_id=config_.calibration.id;
  sample.truth.sequence=packet.sequence;
  sample.truth.capture_tick=tick;
  sample.truth.camera_pose=camera_pose;
  if (!config_.enabled) return sample;
  if (config_.output=="image") {
    packet.image=render_camera(snapshot,camera_pose,config_.calibration,config_.image,
      config_.seed+packet.sequence*0x9e3779b97f4a7c15ULL);
    return sample;
  }
  const auto& k=config_.calibration;
  auto inside=[&](Pixel p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && p.x>=0 && p.x<k.width && p.y>=0 && p.y<k.height;
  };
  auto occluded=[&](Vec3 point,const Surface& self) {
    if (!config_.surface_occlusion) return false;
    const Vec3 ray=point-camera_pose.translation;
    for (const auto& t:snapshot.targets) for (const auto& surface:t.surfaces) {
      if (surface.target_id==self.target_id && surface.plate_id==self.plate_id) continue;
      double denom=dot(ray,surface.normal);
      if (std::abs(denom)<1e-12) continue;
      double u=dot(surface.center-camera_pose.translation,surface.normal)/denom;
      if (u>1e-8 && u<1-1e-8 && surface.contains(camera_pose.translation+ray*u)) return true;
    }
    return false;
  };
  for (const auto& target:snapshot.targets) for (const auto& surface:target.surfaces) {
    TruthFeature annotation;
    annotation.target_id=target.id;
    annotation.plate_id=surface.plate_id;
    FeatureObservation observation;
    observation.covariance={config_.pixel_sigma*config_.pixel_sigma,0,config_.pixel_sigma*config_.pixel_sigma};
    std::vector<Vec3> points;
    auto appearance=config_.appearances.find(target.id);
    if (appearance!=config_.appearances.end()) {
      observation.model_id=appearance->second.model_id;
      observation.measured_class=appearance->second.measured_class;
      for (auto p:appearance->second.points)
        points.push_back(surface.center+surface.right*p.x+surface.up*p.y+surface.normal*p.z);
    } else if (surface.disk_radius>0) {
      observation.model_id="synthetic-disk-cardinals-v1";
      observation.measured_class="disk";
      double r=surface.disk_radius;
      points={surface.center,surface.center+surface.right*r,surface.center+surface.up*r,
        surface.center-surface.right*r,surface.center-surface.up*r};
    } else {
      observation.model_id="synthetic-armor-envelope-v1";
      observation.measured_class="armor";
      const auto corners=surface.corners();
      points.assign(corners.begin(),corners.end());
    }
    // This represents visible activation lighting only; it never publishes the internal state machine.
    if (surface.disk_radius>0 && !surface.active) annotation.reason="unlit";
    else if (dot(surface.normal,camera_pose.translation-surface.center)<=0) annotation.reason="back_face";
    else {
      for (auto world_point:points) {
        const auto p=camera_pose.inverse(world_point);
        if (p.z<=1e-9) { annotation.reason="behind_camera"; break; }
        Pixel pixel{k.fx*p.x/p.z+k.cx,k.fy*p.y/p.z+k.cy};
        annotation.ideal_pixels.push_back(pixel);
        if (!inside(pixel)) annotation.reason="outside_image";
        else if (occluded(world_point,surface)) annotation.reason="surface_occlusion";
      }
    }
    annotation.visible=annotation.reason.empty();
    if (annotation.visible) {
      if (config_.dropout>0 && uniform()<config_.dropout) annotation.reason="dropout";
      else {
        for (auto p:annotation.ideal_pixels) observation.pixels.push_back({p.x+noise(),p.y+noise()});
        if (std::all_of(observation.pixels.begin(),observation.pixels.end(),inside)) {
          annotation.observation_index=static_cast<int>(packet.observations.size());
          packet.observations.push_back(std::move(observation));
          annotation.reason="observed";
        } else annotation.reason="noise_outside_image";
      }
    }
    sample.truth.features.push_back(std::move(annotation));
  }
  // False positives use the same public model vocabulary as real measurements.
  if (config_.false_positive_probability>0 && uniform()<config_.false_positive_probability) {
    FeatureObservation false_positive;
    false_positive.model_id="synthetic-armor-envelope-v1";
    false_positive.measured_class="armor";
    false_positive.covariance={config_.pixel_sigma*config_.pixel_sigma,0,config_.pixel_sigma*config_.pixel_sigma};
    double hw=std::min(8.,k.width*.1),hh=std::min(8.,k.height*.1);
    double x=hw+uniform()*(k.width-2*hw), y=hh+uniform()*(k.height-2*hh);
    false_positive.pixels={{x-hw,y-hh},{x+hw,y-hh},{x+hw,y+hh},{x-hw,y+hh}};
    packet.observations.push_back(std::move(false_positive));
  }
  return sample;
}
} // namespace rm_sim
