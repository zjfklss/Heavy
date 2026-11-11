#pragma once
#include "rm_sim/world.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm_sim {
using Tick = uint64_t;
inline void ensure(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}
inline bool finite(Vec3 p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
// Columns are the child frame's axes expressed in the parent frame.
struct Rotation {
  Vec3 x{1,0,0}, y{0,1,0}, z{0,0,1};
  Vec3 apply(Vec3 p) const { return x*p.x + y*p.y + z*p.z; }
  Vec3 inverse(Vec3 p) const { return {dot(x,p), dot(y,p), dot(z,p)}; }
  void validate() const {
    ensure(finite(x) && finite(y) && finite(z) && std::abs(norm(x)-1)<1e-8 &&
      std::abs(norm(y)-1)<1e-8 && std::abs(norm(z)-1)<1e-8 &&
      norm(cross(x,y)-z)<1e-8, "Rotation must be right-handed and orthonormal");
  }
};
// T_A_B maps points from B into A.
struct Pose {
  Rotation rotation;
  Vec3 translation;
  Vec3 apply(Vec3 p) const { return rotation.apply(p)+translation; }
  Vec3 inverse(Vec3 p) const { return rotation.inverse(p-translation); }
  void validate() const { rotation.validate(); ensure(finite(translation), "Invalid pose translation"); }
};
inline Pose compose(const Pose& a, const Pose& b) {
  return {{a.rotation.apply(b.rotation.x), a.rotation.apply(b.rotation.y),
    a.rotation.apply(b.rotation.z)}, a.apply(b.translation)};
}
inline Rotation yaw_rotation(double yaw) {
  return {{std::cos(yaw), std::sin(yaw),0}, {-std::sin(yaw),std::cos(yaw),0}, {0,0,1}};
}
inline Rotation pitch_rotation(double pitch) {
  return {{std::cos(pitch),0,std::sin(pitch)}, {0,1,0}, {-std::sin(pitch),0,std::cos(pitch)}};
}
inline double wrap_angle(double a) { return std::remainder(a, 2*pi); }
inline Tick quantize_delay(double seconds, double dt) {
  ensure(std::isfinite(dt) && dt>0 && dt<=.001, "Invalid clock timestep");
  ensure(std::isfinite(seconds) && seconds>=0 && seconds<=600, "Delay must be in [0,600] s");
  double ticks = seconds/dt;
  ensure(std::isfinite(ticks) && ticks<=1e12, "Clock range exceeded");
  return static_cast<Tick>(std::ceil(ticks-1e-10));
}
// Every deadline is derived from its ideal epoch, avoiding rounded-period drift.
class PeriodicClock {
 public:
  PeriodicClock(double hz, double dt) : hz_(hz), dt_(dt) {
    ensure(std::isfinite(dt) && dt>0 && dt<=.001 && std::isfinite(hz) && hz>0 && hz*dt<=1,
      "Sampling frequency must be in (0, physics frequency]");
  }
  Tick next() const {
    const long double deadline = static_cast<long double>(index_)/(hz_*dt_);
    ensure(deadline<=1e12, "Periodic clock range exceeded");
    return static_cast<Tick>(std::ceil(deadline-1e-10L));
  }
  bool take(Tick tick) {
    if (tick<next()) return false;
    do { ++index_; } while (next()<=tick);
    return true;
  }
 private:
  double hz_, dt_;
  Tick index_=0;
};
struct Pixel { double x=0, y=0; };
} // namespace rm_sim
