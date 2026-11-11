#include "rm_sim/world.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rm_sim
{
namespace
{
void require(bool ok, const char* message)
{
  if (!ok) throw std::invalid_argument(message);
}
bool finite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
uint64_t random_bits(uint64_t& state)
{
  uint64_t z = (state += 0x9e3779b97f4a7c15ULL);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}
double uniform(uint64_t& state) { return (random_bits(state) >> 11) * 0x1.0p-53; }
bool is_rune(TargetKind kind)
{
  return kind == TargetKind::small_rune || kind == TargetKind::big_rune;
}
Surface rectangle(const std::string& id, int index, Vec3 center, double yaw, double tilt,
                  double width, double height)
{
  Surface s;
  s.target_id = id;
  s.plate_id = index;
  s.center = center;
  s.normal = {std::cos(yaw) * std::cos(tilt), std::sin(yaw) * std::cos(tilt), std::sin(tilt)};
  s.right = {-std::sin(yaw), std::cos(yaw), 0};
  s.up = cross(s.normal, s.right);
  s.width = width;
  s.height = height;
  return s;
}
}  // namespace

double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(Vec3 a) { return std::sqrt(dot(a, a)); }
Vec3 cross(Vec3 a, Vec3 b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vec3 unit(Vec3 a)
{
  require(finite(a) && norm(a) > 1e-12, "Invalid direction");
  return a * (1 / norm(a));
}

void Motion::validate() const
{
  for (double v : {initial, velocity, acceleration, jerk, amplitude, frequency, phase})
    require(std::isfinite(v) && std::abs(v) <= 1e6,
            "Motion parameter outside finite simulation range");
  require(frequency > 0 && frequency <= 100 && amplitude >= 0,
          "Invalid motion amplitude/frequency");
  require(kind >= MotionKind::stationary && kind <= MotionKind::random, "Invalid motion kind");
}
Kinematics Motion::at(double t) const
{
  Kinematics s{initial, 0, 0, 0};
  switch (kind) {
    case MotionKind::stationary:
      break;
    case MotionKind::velocity:
      s = {initial + velocity * t, velocity, 0, 0};
      break;
    case MotionKind::acceleration:
      s = {initial + velocity * t + acceleration * t * t / 2, velocity + acceleration * t,
           acceleration, 0};
      break;
    case MotionKind::jerk:
      s = {initial + velocity * t + acceleration * t * t / 2 + jerk * t * t * t / 6,
           velocity + acceleration * t + jerk * t * t / 2, acceleration + jerk * t, jerk};
      break;
    case MotionKind::sinusoidal: {
      const double w = frequency, q = w * t + phase;
      s = {initial + velocity * t + acceleration * t * t / 2 + amplitude / w * t * std::cos(phase) -
             amplitude / (w * w) * (std::sin(q) - std::sin(phase)),
           velocity + acceleration * t + amplitude / w * (std::cos(phase) - std::cos(q)),
           acceleration + amplitude * std::sin(q), amplitude * w * std::cos(q)};
      break;
    }
    case MotionKind::random: {
      uint64_t rng = seed;
      for (int i = 0; i < 3; ++i) {
        const double w = frequency * (.35 + 1.35 * uniform(rng)), p = 2 * pi * uniform(rng),
                     a = amplitude / 6;
        s.position += a * (std::sin(w * t + p) - std::sin(p));
        s.velocity += a * w * std::cos(w * t + p);
        s.acceleration -= a * w * w * std::sin(w * t + p);
        s.jerk -= a * w * w * w * std::cos(w * t + p);
      }
      break;
    }
  }
  return s;
}

TargetConfig TargetConfig::outpost(std::string id, Vec3 origin)
{
  TargetConfig c;
  c.id = std::move(id);
  c.kind = TargetKind::outpost;
  c.armor_marking = "outpost";
  c.position = origin;
  c.radius = .275;
  c.tilt = -15 * pi / 180;
  return c;
}
TargetConfig TargetConfig::rune(std::string id, Vec3 center, bool big)
{
  TargetConfig c;
  c.id = std::move(id);
  c.kind = big ? TargetKind::big_rune : TargetKind::small_rune;
  c.position = center;
  return c;
}
void TargetConfig::validate() const
{
  require(!id.empty() && finite(position), "Invalid target id/position");
  require(kind >= TargetKind::car && kind <= TargetKind::big_rune, "Invalid target kind");
  require(armor_marking=="1" || armor_marking=="2" || armor_marking=="3" ||
    armor_marking=="4" || armor_marking=="sentry" ||
    (kind==TargetKind::outpost && armor_marking=="outpost"),"Invalid RM2026 armor marking");
  for (auto& m : translation) m.validate();
  rotation.validate();
  for (double v : {armor_width, armor_height, radius, second_radius, height_difference, tilt,
                   ramp_seconds, stop_time, axis_yaw, rune_radius, disk_radius, rune_a, rune_omega})
    require(std::isfinite(v), "Non-finite target parameter");
  for (double z : outpost_heights) require(std::isfinite(z), "Non-finite outpost height");
  require(armor_width > 0 && armor_height > 0 && radius > 0 && second_radius > 0,
          "Invalid armor geometry");
  require(std::abs(tilt) < pi / 2 && std::abs(height_difference) <= .1,
          "Invalid tilt/height difference");
  require(ramp_seconds > 0 && ramp_seconds <= 5 && stop_time >= 0, "Invalid outpost timing");
  require(direction == 1 || direction == -1, "Direction must be +1 or -1");
  require(rune_radius > 0 && disk_radius > 0 && disk_radius < rune_radius, "Invalid rune geometry");
  require(rune_a >= .780 && rune_a <= 1.045 && rune_omega >= 1.884 && rune_omega <= 2,
          "Rune a/omega outside 2027 rule range");
  if (kind != TargetKind::car) {
    for (const auto& m : translation)
      require(m.kind == MotionKind::stationary && m.initial == 0, "Props must have fixed centers");
    require(rotation.kind == MotionKind::stationary, "Props use their rule-defined rotation");
  }
}

std::array<Vec3, 4> Surface::corners() const
{
  return {center - right * (width / 2) + up * (height / 2),
          center + right * (width / 2) + up * (height / 2),
          center + right * (width / 2) - up * (height / 2),
          center - right * (width / 2) - up * (height / 2)};
}
bool Surface::contains(Vec3 point) const
{
  const auto d = point - center;
  const double u = dot(d, right), v = dot(d, up);
  if (disk_radius > 0) return u * u + v * v <= disk_radius * disk_radius + 1e-12;
  return std::abs(u) <= width / 2 + 1e-10 && std::abs(v) <= height / 2 + 1e-10;
}

World::World(WorldConfig config) : config_(config)
{
  require(std::isfinite(config.timestep) && config.timestep > 0 && config.timestep <= .001,
          "Physics timestep must be in (0, 0.001] seconds");
  require(finite(config.gravity) && std::isfinite(config.ground_z), "Invalid world geometry");
  require(std::isfinite(config.minimum_normal_speed) && config.minimum_normal_speed >= 0 &&
            std::isfinite(config.detection_interval) && config.detection_interval >= 0,
          "Invalid hit detection settings");
}
void World::add_target(TargetConfig config)
{
  require(tick_ == 0, "Add targets before stepping the world");
  config.validate();
  for (const auto& t : targets_) require(t.config.id != config.id, "Duplicate target id");
  Target t;
  t.config = std::move(config);
  t.rng = t.config.seed;
  targets_.push_back(t);
  if (is_rune(t.config.kind) && t.config.initially_active) activate_rune(t.config.id);
}
void World::launch(const Shot& shot)
{
  require(
    !shot.id.empty() && finite(shot.origin) && finite(shot.velocity) && norm(shot.velocity) > 0,
    "Invalid shot");
  require(shot.origin.z > config_.ground_z && std::isfinite(shot.lifetime) && shot.lifetime > 0,
          "Shot must start above ground and have positive lifetime");
  for (const auto& p : projectiles_) require(p.shot.id != shot.id, "Duplicate shot id");
  projectiles_.push_back({shot, time(), true});
}
void World::select_group(Target& t, double now)
{
  t.active.fill(false);
  t.first_hit_time = -1;
  t.group_time = now;
  std::vector<int> available;
  for (int i = 0; i < 5; ++i)
    if (t.config.kind == TargetKind::big_rune || !t.completed[i]) available.push_back(i);
  const int count = t.config.kind == TargetKind::big_rune ? 2 : 1;
  for (int j = 0; j < count; ++j) {
    const auto k = static_cast<size_t>(random_bits(t.rng) % available.size());
    t.active[available[k]] = true;
    available.erase(available.begin() + k);
  }
}
void World::activate_rune(const std::string& id)
{
  for (auto& t : targets_)
    if (t.config.id == id) {
      require(is_rune(t.config.kind), "Target is not a rune");
      t.phase_at_activation = rotation(t, time()).position;
      t.activation_time = time();
      t.activation = "activating";
      t.ended_at = -1;
      t.groups = 0;
      t.accepted_hits = 0;
      t.completed.fill(false);
      select_group(t, time());
      return;
    }
  throw std::invalid_argument("Unknown rune id");
}
void World::stop_outpost(const std::string& id)
{
  for (auto& t : targets_)
    if (t.config.id == id) {
      require(t.config.kind == TargetKind::outpost, "Target is not an outpost");
      if (t.stopped_at < 0) {
        t.stopped_phase = rotation(t, time()).position;
        t.stopped_at = time();
      }
      return;
    }
  throw std::invalid_argument("Unknown outpost id");
}
Kinematics World::rotation(const Target& t, double now) const
{
  const auto& c = t.config;
  if (c.kind == TargetKind::car) return c.rotation.at(now);
  if (c.kind == TargetKind::outpost) {
    if (t.stopped_at >= 0 && now >= t.stopped_at) return {t.stopped_phase, 0, 0, 0};
    const double u = std::min(now, c.stop_time), w = c.direction * .8 * pi;
    const double phase = c.rotation.initial + w * (u < c.ramp_seconds ? u * u / (2 * c.ramp_seconds)
                                                                      : u - c.ramp_seconds / 2);
    if (now >= c.stop_time) return {phase, 0, 0, 0};
    return {phase, w * std::min(u / c.ramp_seconds, 1.0),
            u < c.ramp_seconds ? w / c.ramp_seconds : 0, 0};
  }
  const double w = c.direction * pi / 3;
  if (t.ended_at >= 0 && now >= t.ended_at)
    return {t.ended_phase + w * (now - t.ended_at), w, 0, 0};
  if (t.activation != "activating" || c.kind == TargetKind::small_rune)
    return {c.rotation.initial + w * now, w, 0, 0};
  const double u = now - t.activation_time, a = c.rune_a, f = c.rune_omega, b = 2.090 - a;
  return {t.phase_at_activation + c.direction * (a / f * (1 - std::cos(f * u)) + b * u),
          c.direction * (a * std::sin(f * u) + b), c.direction * a * f * std::cos(f * u),
          -c.direction * a * f * f * std::sin(f * u)};
}
TargetState World::state(const Target& t, double now) const
{
  TargetState s;
  const auto& c = t.config;
  s.id = c.id;
  s.kind = c.kind;
  s.armor_marking = c.kind==TargetKind::outpost ? "outpost":c.armor_marking;
  s.activation = t.activation;
  s.completed_groups = t.groups;
  s.accepted_hits = t.accepted_hits;
  const auto x = c.translation[0].at(now), y = c.translation[1].at(now),
             z = c.translation[2].at(now);
  s.position = c.position + Vec3{x.position, y.position, z.position};
  s.velocity = {x.velocity, y.velocity, z.velocity};
  s.acceleration = {x.acceleration, y.acceleration, z.acceleration};
  s.jerk = {x.jerk, y.jerk, z.jerk};
  s.rotation = rotation(t, now);
  const int count = c.kind == TargetKind::car ? 4 : (c.kind == TargetKind::outpost ? 3 : 5);
  for (int i = 0; i < count; ++i) {
    const double phase = s.rotation.position + 2 * pi * i / count;
    if (is_rune(c.kind)) {
      const Vec3 normal{std::cos(c.axis_yaw), std::sin(c.axis_yaw), 0};
      const Vec3 right{-std::sin(c.axis_yaw), std::cos(c.axis_yaw), 0}, up{0, 0, 1};
      // Angle zero points up. Positive rotation follows the right-hand rule about normal.
      Surface p;
      p.target_id = c.id;
      p.plate_id = i;
      p.normal = normal;
      p.center = s.position + (up * std::cos(phase) - right * std::sin(phase)) * c.rune_radius;
      p.right = right * std::cos(phase) + up * std::sin(phase);
      p.up = up * std::cos(phase) - right * std::sin(phase);
      p.width = p.height = 2 * c.disk_radius;
      p.disk_radius = c.disk_radius;
      p.active = t.activation == "activating" && t.active[i];
      s.surfaces.push_back(p);
    } else {
      const double radius = c.kind == TargetKind::car && i % 2 ? c.second_radius : c.radius;
      const double height =
        c.kind == TargetKind::outpost ? c.outpost_heights[i] : (i % 2 ? c.height_difference : 0);
      const Vec3 center =
        s.position + Vec3{radius * std::cos(phase), radius * std::sin(phase), height};
      s.surfaces.push_back(
        rectangle(c.id, i, center, phase, c.tilt, c.armor_width, c.armor_height));
    }
  }
  return s;
}
void World::finish_attempt(Target& t, double now, const std::string& status)
{
  t.ended_phase = rotation(t, now).position;
  t.ended_at = now;
  t.activation = status;
  t.active.fill(false);
}
void World::update_activation(Target& t, double now)
{
  if (!is_rune(t.config.kind) || t.activation != "activating") return;
  if (now - t.activation_time >= 20 - 1e-10) {
    finish_attempt(t, now, "expired");
    return;
  }
  if (t.first_hit_time >= 0 && now - t.first_hit_time >= 1 - 1e-10) {
    if (t.groups >= 5)
      finish_attempt(t, now, "activated");
    else
      select_group(t, now);
  } else if (t.first_hit_time < 0 && now - t.group_time > 2.5 + 1e-10) {
    // Rule 5.6.3.1: failed sequence resets progress inside the same 20 s attempt.
    t.groups = 0;
    t.accepted_hits = 0;
    t.completed.fill(false);
    select_group(t, now);
  }
}
void World::accept_hit(Target& t, Hit& h)
{
  update_activation(t, h.time);
  if (h.normal_speed <= config_.minimum_normal_speed) {
    h.reason = "normal_speed_too_low";
    return;
  }
  if (h.time - t.last_detection[h.plate_id] < config_.detection_interval - 1e-10) {
    h.reason = "detection_interval";
    return;
  }
  t.last_detection[h.plate_id] = h.time;
  if (is_rune(t.config.kind)) {
    if (t.activation != "activating") {
      h.reason = "rune_not_activating";
      return;
    }
    if (!t.active[h.plate_id]) {
      h.reason = "inactive_plate_reset";
      t.groups = 0;
      t.accepted_hits = 0;
      t.completed.fill(false);
      select_group(t, h.time);
      return;
    }
    t.active[h.plate_id] = false;
    ++t.accepted_hits;
    if (t.config.kind == TargetKind::small_rune) {
      t.completed[h.plate_id] = true;
      ++t.groups;
      if (t.groups == 5)
        finish_attempt(t, h.time, "activated");
      else
        select_group(t, h.time);
    } else if (t.first_hit_time < 0) {
      t.first_hit_time = h.time;
      ++t.groups;
    } else {
      if (t.groups == 5)
        finish_attempt(t, h.time, "activated");
      else
        select_group(t, h.time);
    }
  }
  h.effective = true;
  h.reason = "accepted";
}
Vec3 World::projectile_position(const Projectile& p, double t) const
{
  const double dt = t - p.born;
  return p.shot.origin + p.shot.velocity * dt + config_.gravity * (dt * dt / 2);
}
Vec3 World::projectile_velocity(const Projectile& p, double t) const
{
  return p.shot.velocity + config_.gravity * (t - p.born);
}
void World::step(uint64_t ticks)
{
  require(ticks <= 10000000, "Too many ticks in one call");
  for (uint64_t tick = 0; tick < ticks; ++tick) {
    const double begin = time(), end = (tick_ + 1) * config_.timestep;
    struct Candidate
    {
      Hit hit;
      size_t projectile, target;
    };
    std::vector<Candidate> candidates;
    for (size_t ip = 0; ip < projectiles_.size(); ++ip) {
      auto& p = projectiles_[ip];
      if (!p.alive) continue;
      double until = std::min(end, p.born + p.shot.lifetime);
      // Floor is an absorbing boundary; it competes with armor for earliest contact.
      if (projectile_position(p, until).z <= config_.ground_z) {
        double lo = begin, hi = until;
        for (int j = 0; j < 35; ++j) {
          const double mid = (lo + hi) / 2;
          if (projectile_position(p, mid).z > config_.ground_z)
            lo = mid;
          else
            hi = mid;
        }
        until = hi;
      }
      Candidate best;
      best.hit.time = std::numeric_limits<double>::infinity();
      for (size_t it = 0; it < targets_.size(); ++it) {
        const auto& target = targets_[it];
        // Bound angular sweep per search bracket as well as the 1 ms physics tick.
        const double w = std::max(std::abs(rotation(target, begin).velocity),
                                  std::abs(rotation(target, until).velocity));
        const double brackets = std::ceil(w * (until - begin) / .01);
        require(std::isfinite(brackets) && brackets <= 1000,
                "Angular speed too high for collision solver");
        const int slices = std::max(1, static_cast<int>(brackets));
        for (int k = 0; k < slices; ++k) {
          const double ta = begin + (until - begin) * k / slices,
                       tb = begin + (until - begin) * (k + 1) / slices;
          const auto sa = state(target, ta), sb = state(target, tb);
          for (size_t i = 0; i < sa.surfaces.size(); ++i) {
            const double da =
              dot(projectile_position(p, ta) - sa.surfaces[i].center, sa.surfaces[i].normal);
            const double db =
              dot(projectile_position(p, tb) - sb.surfaces[i].center, sb.surfaces[i].normal);
            if (da < 0 || db > 0 || da == db) continue;  // Front face only.
            double lo = ta, hi = tb;
            for (int j = 0; j < 32; ++j) {
              const double mid = (lo + hi) / 2;
              const auto sm = state(target, mid).surfaces[i];
              if (dot(projectile_position(p, mid) - sm.center, sm.normal) > 0)
                lo = mid;
              else
                hi = mid;
            }
            const double th = (lo + hi) / 2;
            const auto s = state(target, th).surfaces[i];
            const auto point = projectile_position(p, th);
            if (!s.contains(point) || th >= best.hit.time || point.z < config_.ground_z) continue;
            const auto d = point - s.center;
            const double u = dot(d, s.right), v = dot(d, s.up), eps = 1e-6;
            const auto before = state(target, th - eps).surfaces[i],
                       after = state(target, th + eps).surfaces[i];
            const Vec3 surface_velocity = ((after.center + after.right * u + after.up * v) -
                                           (before.center + before.right * u + before.up * v)) *
                                          (.5 / eps);
            Hit h;
            h.time = th;
            h.shot_id = p.shot.id;
            h.target_id = target.config.id;
            h.plate_id = static_cast<int>(i);
            h.point = point;
            h.plate_point = Vec3{u, v, dot(d, s.normal)};
            h.normal_speed = -dot(projectile_velocity(p, th) - surface_velocity, s.normal);
            // Equal 15 mm radial bands: Fig. 5-17, 300 mm effective diameter.
            h.ring =
              s.disk_radius > 0
                ? std::clamp(
                    10 - static_cast<int>(std::floor(std::hypot(u, v) / (s.disk_radius / 10))), 1,
                    10)
                : 0;
            best = {h, ip, it};
          }
        }
      }
      if (std::isfinite(best.hit.time))
        candidates.push_back(best);
      else if (until < end || end >= p.born + p.shot.lifetime ||
               projectile_position(p, end).z <= config_.ground_z)
        p.alive = false;
    }
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const auto& a, const auto& b) { return a.hit.time < b.hit.time; });
    for (auto& c : candidates) {
      accept_hit(targets_[c.target], c.hit);
      hits_.push_back(c.hit);
      projectiles_[c.projectile].alive =
        false;  // Physical impact consumes even a non-scoring shot.
    }
    for (auto& t : targets_) update_activation(t, end);
    ++tick_;
  }
}
Snapshot World::snapshot() const
{
  Snapshot s;
  s.time = time();
  for (const auto& t : targets_) s.targets.push_back(state(t, time()));
  for (const auto& p : projectiles_)
    if (p.alive)
      s.projectiles.push_back(
        {p.shot.id, projectile_position(p, time()), projectile_velocity(p, time())});
  return s;
}
std::string to_string(TargetKind k)
{
  switch (k) {
    case TargetKind::car:
      return "car";
    case TargetKind::outpost:
      return "outpost";
    case TargetKind::small_rune:
      return "small_rune";
    case TargetKind::big_rune:
      return "big_rune";
  }
  throw std::invalid_argument("Invalid target kind");
}
}  // namespace rm_sim
