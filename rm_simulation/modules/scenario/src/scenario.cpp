#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <set>
#include <stdexcept>

#include "rm_sim/scenario.hpp"

using nlohmann::json;
using namespace rm_sim;
namespace rm_sim::scenario
{
void keys(const json &j, std::initializer_list<const char *> allowed)
{
  if (!j.is_object()) throw std::invalid_argument("Expected a JSON object");
  std::set<std::string> names;
  for (auto s : allowed) names.insert(s);
  for (auto it = j.begin(); it != j.end(); ++it)
    if (!names.count(it.key())) throw std::invalid_argument("Unknown field: " + it.key());
}
Vec3 vec(const json &j)
{
  if (!j.is_array() || j.size() != 3) throw std::invalid_argument("Expected [x,y,z]");
  Vec3 v{j[0].get<double>(), j[1].get<double>(), j[2].get<double>()};
  if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
    throw std::invalid_argument("Non-finite vector");
  return v;
}
json js(Vec3 v) { return json::array({v.x, v.y, v.z}); }
json js(Kinematics k) { return {k.position, k.velocity, k.acceleration, k.jerk}; }
uint64_t seed(const json &j, const char *key, uint64_t fallback)
{
  if (!j.contains(key)) return fallback;
  if (!j[key].is_number_unsigned())
    throw std::invalid_argument("Seed must be a nonnegative integer");
  return j[key].get<uint64_t>();
}
Motion motion(const json &j)
{
  keys(j, {"kind", "initial", "velocity", "acceleration", "jerk", "amplitude", "frequency", "phase",
           "seed"});
  Motion m;
  const auto k = j.value("kind", std::string("stationary"));
  if (k == "stationary")
    m.kind = MotionKind::stationary;
  else if (k == "velocity")
    m.kind = MotionKind::velocity;
  else if (k == "acceleration")
    m.kind = MotionKind::acceleration;
  else if (k == "jerk")
    m.kind = MotionKind::jerk;
  else if (k == "sinusoidal")
    m.kind = MotionKind::sinusoidal;
  else if (k == "random")
    m.kind = MotionKind::random;
  else
    throw std::invalid_argument("Unknown motion kind: " + k);
  m.initial = j.value("initial", m.initial);
  m.velocity = j.value("velocity", m.velocity);
  m.acceleration = j.value("acceleration", m.acceleration);
  m.jerk = j.value("jerk", m.jerk);
  m.amplitude = j.value("amplitude", m.amplitude);
  m.frequency = j.value("frequency", m.frequency);
  m.phase = j.value("phase", m.phase);
  m.seed = seed(j, "seed", m.seed);
  m.validate();
  return m;
}
TargetConfig target(const json &j)
{
  keys(j, {"id",
           "armor_marking",
           "kind",
           "position",
           "translation",
           "rotation",
           "armor_width",
           "armor_height",
           "radius",
           "second_radius",
           "height_difference",
           "tilt",
           "outpost_heights",
           "ramp_seconds",
           "stop_time",
           "direction",
           "axis_yaw",
           "rune_radius",
           "disk_radius",
           "rune_a",
           "rune_omega",
           "seed",
           "initially_active"});
  const auto k = j.at("kind").get<std::string>(), id = j.at("id").get<std::string>();
  const auto p = vec(j.at("position"));
  TargetConfig c;
  if (k == "car") {
    c.id = id;
    c.position = p;
  } else if (k == "outpost")
    c = TargetConfig::outpost(id, p);
  else if (k == "small_rune" || k == "big_rune")
    c = TargetConfig::rune(id, p, k == "big_rune");
  else
    throw std::invalid_argument("Unknown target kind: " + k);
  if (j.contains("translation")) {
    const auto &a = j["translation"];
    if (!a.is_array() || a.size() != 3)
      throw std::invalid_argument("translation needs three axis motions");
    for (int i = 0; i < 3; ++i) c.translation[i] = motion(a[i]);
  }
  if (j.contains("rotation")) c.rotation = motion(j["rotation"]);
#define PARAM(name) c.name = j.value(#name, c.name)
  PARAM(armor_width);
  PARAM(armor_marking);
  PARAM(armor_height);
  PARAM(radius);
  PARAM(second_radius);
  PARAM(height_difference);
  PARAM(tilt);
  PARAM(ramp_seconds);
  PARAM(stop_time);
  PARAM(axis_yaw);
  PARAM(rune_radius);
  PARAM(disk_radius);
  PARAM(rune_a);
  PARAM(rune_omega);
  PARAM(initially_active);
#undef PARAM
  if (j.contains("direction") && !j["direction"].is_number_integer())
    throw std::invalid_argument("direction must be integer");
  c.direction = j.value("direction", c.direction);
  c.seed = seed(j, "seed", c.seed);
  if (j.contains("outpost_heights")) {
    const auto a = vec(j["outpost_heights"]);
    c.outpost_heights = {a.x, a.y, a.z};
  }
  c.validate();
  return c;
}
uint64_t ticks(double t, double dt)
{
  if (!std::isfinite(t) || t < 0 || t > 600)
    throw std::invalid_argument("Time must be within [0,600] s");
  if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(t / dt) || t / dt > 1e12)
    throw std::invalid_argument("Time exceeds supported tick range");
  const auto n = static_cast<uint64_t>(std::llround(t / dt));
  if (std::abs(n * dt - t) > 1e-8)
    throw std::invalid_argument("Time must align to physics timestep");
  return n;
}
json frame(const Snapshot &s)
{
  json j = {{"time", s.time}, {"targets", json::array()}, {"projectiles", json::array()}};
  for (const auto &t : s.targets) {
    json x = {{"id", t.id},
              {"armor_marking", t.armor_marking},
              {"kind", to_string(t.kind)},
              {"position", js(t.position)},
              {"velocity", js(t.velocity)},
              {"acceleration", js(t.acceleration)},
              {"jerk", js(t.jerk)},
              {"rotation", js(t.rotation)},
              {"activation", t.activation},
              {"completed_groups", t.completed_groups},
              {"accepted_hits", t.accepted_hits},
              {"surfaces", json::array()}};
    for (const auto &p : t.surfaces) {
      json corners = json::array();
      for (const auto &v : p.corners()) corners.push_back(js(v));
      x["surfaces"].push_back({{"plate_id", p.plate_id},
                               {"center", js(p.center)},
                               {"normal", js(p.normal)},
                               {"right", js(p.right)},
                               {"up", js(p.up)},
                               {"width", p.width},
                               {"height", p.height},
                               {"disk_radius", p.disk_radius},
                               {"active", p.active},
                               {"corners", corners}});
    }
    j["targets"].push_back(x);
  }
  for (const auto &p : s.projectiles)
    j["projectiles"].push_back(
      {{"id", p.id}, {"position", js(p.position)}, {"velocity", js(p.velocity)}});
  return j;
}
}  // namespace
