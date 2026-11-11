#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rm_sim
{
constexpr double pi = 3.14159265358979323846;
struct Vec3
{
  double x = 0, y = 0, z = 0;
  Vec3 operator+(Vec3 b) const { return {x + b.x, y + b.y, z + b.z}; }
  Vec3 operator-(Vec3 b) const { return {x - b.x, y - b.y, z - b.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
};
double dot(Vec3 a, Vec3 b);
double norm(Vec3 a);
Vec3 cross(Vec3 a, Vec3 b);
Vec3 unit(Vec3 a);

enum class MotionKind
{
  stationary,
  velocity,
  acceleration,
  jerk,
  sinusoidal,
  random
};
struct Kinematics
{
  double position = 0, velocity = 0, acceleration = 0, jerk = 0;
};
// SI units. Random motion is a bounded, seeded sum of smooth harmonics, not white noise.
struct Motion
{
  MotionKind kind = MotionKind::stationary;
  double initial = 0, velocity = 0, acceleration = 0, jerk = 0;
  double amplitude = 1, frequency = 1, phase = 0;
  uint64_t seed = 1;
  Kinematics at(double time) const;
  void validate() const;
};

enum class TargetKind
{
  car,
  outpost,
  small_rune,
  big_rune
};
struct TargetConfig
{
  std::string id;
  std::string armor_marking = "3";
  TargetKind kind = TargetKind::car;
  // World is right-handed: x forward, y left, z up. Rotation is right-handed.
  Vec3 position{4, 0, .3};
  std::array<Motion, 3> translation{};
  Motion rotation{};
  // 2026 armor envelope, explicitly distinct from lightbar/PnP dimensions.
  double armor_width = .135, armor_height = .125;
  double radius = .25, second_radius = .25, height_difference = .05;
  double tilt = 15 * pi / 180;
  // Outpost: heights relative to position.z, H -> M -> L in increasing geometric phase.
  // This is the fixed mounting order; changing motor direction does not mirror the assembly.
  std::array<double, 3> outpost_heights{1.318, 1.216, 1.114};
  double ramp_seconds = 5, stop_time = 180;
  int direction = 1;
  // Rune: disk centers on a 1.4 m diameter circle; front faces axis_yaw.
  double axis_yaw = pi, rune_radius = .7, disk_radius = .15;
  double rune_a = .9, rune_omega = 1.95;
  uint64_t seed = 1;
  // Explicit local practice activation; this is not a complete match referee.
  bool initially_active = true;
  void validate() const;
  static TargetConfig outpost(std::string id, Vec3 pedestal_origin);
  static TargetConfig rune(std::string id, Vec3 center, bool big);
};

struct Surface
{
  std::string target_id;
  int plate_id = 0;
  Vec3 center, normal, right, up;
  double width = 0, height = 0, disk_radius = 0;
  bool active = true;
  std::array<Vec3, 4> corners() const;
  bool contains(Vec3 point) const;
};
struct TargetState
{
  std::string id, activation;
  std::string armor_marking = "3";
  TargetKind kind;
  Vec3 position, velocity, acceleration, jerk;
  Kinematics rotation;
  int completed_groups = 0, accepted_hits = 0;
  std::vector<Surface> surfaces;
};
struct Shot
{
  std::string id;
  Vec3 origin, velocity;
  double lifetime = 3;
};
struct ProjectileState
{
  std::string id;
  Vec3 position, velocity;
};
struct Hit
{
  double time = 0;
  std::string shot_id, target_id, reason;
  int plate_id = 0, ring = 0;
  Vec3 point;
  double normal_speed = 0;
  bool effective = false;
  // Impact position in the struck plate's (right, up, outward normal) frame, metres.
  // This is evaluation data, never an algorithm input.
  std::optional<Vec3> plate_point;
};
struct Snapshot
{
  double time = 0;
  std::vector<TargetState> targets;
  std::vector<ProjectileState> projectiles;
};
struct WorldConfig
{
  double timestep = .001;
  Vec3 gravity{0, 0, -9.81};
  double ground_z = 0;
  // 17 mm normal-impact velocity and same-module detection interval.
  double minimum_normal_speed = 12, detection_interval = .05;
};

class World
{
public:
  explicit World(WorldConfig config = {});
  void add_target(TargetConfig config);
  void launch(const Shot& shot);
  void activate_rune(const std::string& id);
  void stop_outpost(const std::string& id);
  void step(uint64_t ticks = 1);
  Snapshot snapshot() const;
  double time() const { return tick_ * config_.timestep; }
  const std::vector<Hit>& hits() const { return hits_; }

private:
  struct Target
  {
    TargetConfig config;
    uint64_t rng = 1;
    double activation_time = 0, group_time = 0, first_hit_time = -1;
    double phase_at_activation = 0, stopped_at = -1, stopped_phase = 0;
    double ended_at = -1, ended_phase = 0;
    int groups = 0, accepted_hits = 0;
    std::string activation = "idle";
    std::array<bool, 5> active{}, completed{};
    std::array<double, 5> last_detection{{-1e30, -1e30, -1e30, -1e30, -1e30}};
  };
  struct Projectile
  {
    Shot shot;
    double born;
    bool alive = true;
  };
  WorldConfig config_;
  uint64_t tick_ = 0;
  std::vector<Target> targets_;
  std::vector<Projectile> projectiles_;
  std::vector<Hit> hits_;
  Kinematics rotation(const Target& target, double t) const;
  TargetState state(const Target& target, double t) const;
  void select_group(Target& target, double t);
  void update_activation(Target& target, double t);
  void finish_attempt(Target& target, double t, const std::string& status);
  void accept_hit(Target& target, Hit& hit);
  Vec3 projectile_position(const Projectile& p, double t) const;
  Vec3 projectile_velocity(const Projectile& p, double t) const;
};
std::string to_string(TargetKind kind);
}  // namespace rm_sim
