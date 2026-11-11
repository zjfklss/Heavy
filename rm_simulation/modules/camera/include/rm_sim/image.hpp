#pragma once
#include "rm_sim/protocol.hpp"
#include <memory>

namespace rm_sim {
// Owned BGR8 camera pixels. No geometry, semantic labels or target identities.
struct Image {
  int width=0, height=0;
  std::vector<uint8_t> bgr;
  // Rendering diagnostics for the simulator UI; not part of the algorithm protocol.
  std::string renderer="cpu",device="CPU";
};
struct ImageConfig {
  std::string color="blue";
  std::string renderer="auto"; // auto: hardware GPU preferred; gpu: required; cpu: native software.
  double light_length=.056, light_width=.010;
  double small_light_spacing=.135, big_light_spacing=.230;
  double noise_sigma=0;
  void validate() const;
};
struct CameraCalibration;
std::shared_ptr<const Image> render_camera(const Snapshot&,const Pose&,const CameraCalibration&,
  const ImageConfig&,uint64_t seed);
} // namespace rm_sim
