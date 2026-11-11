#include "rm_sim/world.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace rm_sim;
namespace
{
int checks = 0;
void check(bool ok, const char *message)
{
  ++checks;
  if (!ok) throw std::runtime_error(message);
}
void near(double a, double b, double tol = 1e-7)
{
  check(std::abs(a - b) <= tol, "numeric mismatch");
}
void rejects(const std::function<void()> &fn)
{
  bool threw = false;
  try {
    fn();
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "invalid input was accepted");
}
World vacuum()
{
  WorldConfig c;
  c.gravity = {0, 0, 0};
  return World(c);
}
TargetConfig car(std::string id = "car")
{
  TargetConfig c;
  c.id = id;
  return c;
}
void motion_tests()
{
  Motion m;
  m.kind = MotionKind::jerk;
  m.initial = 2;
  m.velocity = 3;
  m.acceleration = 4;
  m.jerk = 6;
  auto k = m.at(2);
  near(k.position, 24);
  near(k.velocity, 23);
  near(k.acceleration, 16);
  near(k.jerk, 6);
  m.kind = MotionKind::sinusoidal;
  m.phase = .4;
  m.frequency = 1.7;
  const double t = .8, h = 1e-5;
  k = m.at(t);
  near((m.at(t + h).position - m.at(t - h).position) / (2 * h), k.velocity, 1e-6);
  near((m.at(t + h).velocity - m.at(t - h).velocity) / (2 * h), k.acceleration, 1e-6);
  near((m.at(t + h).acceleration - m.at(t - h).acceleration) / (2 * h), k.jerk, 1e-6);
  m.kind = MotionKind::random;
  m.seed = 27;
  m.amplitude = 2;
  near(m.at(0).position, m.initial);
  for (int i = 0; i < 1000; ++i) {
    check(std::abs(m.at(i * .1).position - m.initial) <= 2, "random position out of bounds");
    near(m.at(i * .1).position, m.at(i * .1).position, 0);
  }
  k = m.at(t);
  near((m.at(t + h).position - m.at(t - h).position) / (2 * h), k.velocity, 1e-6);
  const double v = m.at(2).position;
  m.seed = 28;
  check(v != m.at(2).position, "seed has no effect");
  m.frequency = 0;
  rejects([&] { m.validate(); });
}
void geometry_tests()
{
  auto w = vacuum();
  auto c = car();
  c.radius = .2;
  c.second_radius = .3;
  w.add_target(c);
  const auto s = w.snapshot().targets.front();
  check(s.surfaces.size() == 4, "car plate count");
  near(norm(s.surfaces[0].center - s.position), .2);
  near(s.surfaces[1].center.z - s.position.z, .05);
  for (const auto &p : s.surfaces) {
    near(norm(p.normal), 1);
    near(dot(p.normal, p.right), 0);
    near(dot(p.normal, p.up), 0);
    near(dot(cross(p.right, p.up), p.normal), 1);
    near(norm(p.corners()[1] - p.corners()[0]), .135);
    near(norm(p.corners()[2] - p.corners()[1]), .125);
    check(p.contains(p.center + p.right * .0675), "rectangle edge rejected");
    check(!p.contains(p.center + p.right * .068), "rectangle miss accepted");
  }
  auto o = TargetConfig::outpost("outpost", {6, 0, .3});
  w.add_target(o);
  const auto out = w.snapshot().targets[1];
  check(out.surfaces.size() == 3, "outpost plate count");
  near(out.surfaces[0].center.z, 1.618);
  near(out.surfaces[1].center.z, 1.516);
  near(out.surfaces[2].center.z, 1.414);
  // A reversed height winding makes the three-plate estimator unable to reconstruct H/M/L.
  // Check the physical mounting order independently of spin direction and initial phase.
  for(int direction:{-1,1}) {
    auto assembly=vacuum();auto config=TargetConfig::outpost("assembly",{6,0,.3});
    config.direction=direction;config.rotation.initial=.7;assembly.add_target(config);assembly.step(6300);
    const auto state=assembly.snapshot().targets.front();
    auto h=state.surfaces[0].center-state.position;
    auto m=state.surfaces[1].center-state.position;
    auto l=state.surfaces[2].center-state.position;
    check(h.z>m.z && m.z>l.z,"Outpost height winding is reversed");
    check(cross(h,m).z>0 && cross(m,l).z>0 && cross(l,h).z>0,
      "Outpost high/middle/low mounting phases were mirrored");
  }
  near(std::hypot(out.surfaces[0].center.x - 6, out.surfaces[0].center.y), .275);
  check(out.surfaces[0].normal.z < 0, "outpost tilt wrong");
  w.add_target(TargetConfig::rune("rune", {9, 0, 2.6}, false));
  const auto rune = w.snapshot().targets[2];
  check(rune.surfaces.size() == 5, "rune plate count");
  int active = 0;
  for (const auto &p : rune.surfaces) {
    near(norm(p.center - rune.position), .7);
    near(p.disk_radius, .15);
    active += p.active;
    check(!p.contains(p.center + p.right * .14 + p.up * .14), "disk treated as rectangle");
  }
  check(active == 1, "small rune needs one active plate");
}
void rule_motion_tests()
{
  auto w = vacuum();
  auto o = TargetConfig::outpost("o", {7, 0, .3});
  o.direction = -1;
  w.add_target(o);
  w.step(2500);
  near(w.snapshot().targets[0].rotation.velocity, -.4 * pi);
  w.step(2500);
  near(w.snapshot().targets[0].rotation.velocity, -.8 * pi);
  w.stop_outpost("o");
  double phase = w.snapshot().targets[0].rotation.position;
  w.step(100);
  near(w.snapshot().targets[0].rotation.position, phase);
  near(w.snapshot().targets[0].rotation.velocity, 0);
  auto stop = vacuum();
  o.stop_time = .05;
  stop.add_target(o);
  stop.step(100);
  near(stop.snapshot().targets[0].rotation.velocity, 0);
  auto r = vacuum();
  auto c = TargetConfig::rune("big", {8, 0, 2.6}, true);
  r.add_target(c);
  r.step(500);
  auto k = r.snapshot().targets[0].rotation;
  near(k.velocity, .9 * std::sin(1.95 * .5) + 1.19);
  near(k.position, .9 / 1.95 * (1 - std::cos(1.95 * .5)) + 1.19 * .5);
  const double before = k.position;
  r.activate_rune("big");
  near(r.snapshot().targets[0].rotation.position, before);
  r.step(20000);
  check(r.snapshot().targets[0].activation == "expired", "rune did not expire");
  near(r.snapshot().targets[0].rotation.velocity, pi / 3);
}
void collision_tests()
{
  auto w = vacuum();
  w.add_target(car());
  w.launch({"hit", {0, 0, .3}, {20, 0, 0}, 1});
  w.step(300);
  check(w.hits().size() == 1 && w.hits()[0].effective, "center hit missing");
  near(w.hits()[0].time, .1875, 1e-9);
  near(w.hits()[0].point.x, 3.75);
  check(w.hits()[0].plate_id == 2, "wrong plate hit");
  check(w.hits()[0].plate_point.has_value(),"impact has no plate coordinates");
  near(norm(*w.hits()[0].plate_point),0);
  auto rotating=vacuum();auto rotating_config=car();
  rotating_config.rotation.kind=MotionKind::velocity;rotating_config.rotation.velocity=.8;
  rotating_config.translation[1].kind=MotionKind::velocity;rotating_config.translation[1].velocity=.6;
  rotating.add_target(rotating_config);
  auto future=rotating;future.step(10);
  const auto plate=future.snapshot().targets[0].surfaces[2];
  const auto impact=plate.center+plate.right*.02+plate.up*.025;
  rotating.launch({"off-center",impact+plate.normal*.2,plate.normal*-20,1});
  rotating.step(11);
  check(rotating.hits().size()==1,"off-center moving impact missing");
  const auto& contact=rotating.hits()[0];
  near(contact.time,.01,1e-9);near(norm(contact.point-impact),0);
  check(contact.plate_point.has_value(),"moving impact has no local coordinates");
  near(contact.plate_point->x,.02);near(contact.plate_point->y,.025);near(contact.plate_point->z,0);
  auto miss = vacuum();
  miss.add_target(car());
  miss.launch({"miss", {0, .5, .3}, {20, 0, 0}, 1});
  miss.step(300);
  check(miss.hits().empty(), "false hit");
  auto back = vacuum();
  back.add_target(car());
  back.launch({"back", {4, 0, .3}, {20, 0, 0}, 1});
  back.step(100);
  check(back.hits().empty(), "back face scored");
  auto slow = vacuum();
  slow.add_target(car());
  slow.launch({"slow", {0, 0, .3}, {10, 0, 0}, 1});
  slow.step(500);
  check(slow.hits().size() == 1 && !slow.hits()[0].effective, "low normal velocity scored");
  auto multi = vacuum();
  multi.add_target(car());
  multi.launch({"a", {0, 0, .3}, {20, 0, 0}, 1});
  multi.launch({"b", {0, 0, .3}, {20, 0, 0}, 1});
  multi.step(300);
  check(multi.hits().size() == 2 && multi.hits()[0].effective && !multi.hits()[1].effective,
        "module debounce failed");
  auto nearest = vacuum();
  auto near_car = car("near"), far_car = car("far");
  far_car.position.x = 6;
  nearest.add_target(far_car);
  nearest.add_target(near_car);
  nearest.launch({"nearest", {0, 0, .3}, {20, 0, 0}, 1});
  nearest.step(400);
  check(nearest.hits().size() == 1 && nearest.hits()[0].target_id == "near",
        "nearest contact not selected");
  auto moving = vacuum();
  auto c = car();
  c.translation[1].kind = MotionKind::velocity;
  c.translation[1].velocity = 1;
  moving.add_target(c);
  moving.launch({"moving", {0, .1875, .3}, {20, 0, 0}, 1});
  moving.step(300);
  check(moving.hits().size() == 1 && moving.hits()[0].effective, "moving target missed");
  auto fast = vacuum();
  fast.add_target(car());
  fast.launch({"fast", {0, 0, .3}, {10000, 0, 0}, 1});
  fast.step();
  check(fast.hits().size() == 1, "fast projectile tunneled");
  World gravity;
  gravity.add_target(car());
  gravity.launch({"ballistic", {0, 0, .3}, {23, 0, 9.81 * 3.75 / (2 * 23)}, 1});
  gravity.step(300);
  check(gravity.hits().size() == 1 && gravity.hits()[0].effective, "gravity trajectory missed");
  World floor;
  c = car();
  c.position.z = -1;
  floor.add_target(c);
  floor.launch({"floor", {0, 0, .1}, {20, 0, -5}, 1});
  floor.step(300);
  check(floor.hits().empty() && floor.snapshot().projectiles.empty(), "floor not absorbing");
}
void fire_at_rune(World &w, int serial, bool active = true)
{
  World future = w;
  future.step(10);
  const auto snapshot = future.snapshot();
  for (const auto &p : snapshot.targets[0].surfaces)
    if (p.active == active) {
      w.launch({"r" + std::to_string(serial), p.center + p.normal * .2, p.normal * -20, 1});
      w.step(11);
      return;
    }
  throw std::runtime_error("no eligible test plate");
}
void rune_tests()
{
  auto small = vacuum();
  small.add_target(TargetConfig::rune("small", {5, 0, 2.6}, false));
  for (int i = 0; i < 5; ++i) fire_at_rune(small, i);
  check(small.hits().size() == 5, "small rune contacts missing");
  check(small.snapshot().targets[0].activation == "activated",
        "five small plates did not activate");
  for (const auto &h : small.hits()) {
    check(h.effective, "small rune hit ineffective");
    check(h.ring == 10, "center ring wrong");
  }
  auto wrong = vacuum();
  wrong.add_target(TargetConfig::rune("small", {5, 0, 2.6}, false));
  fire_at_rune(wrong, 0);
  wrong.step(60);
  fire_at_rune(wrong, 1, false);
  check(wrong.hits().back().reason == "inactive_plate_reset", "inactive hit not identified");
  check(wrong.snapshot().targets[0].completed_groups == 0, "inactive hit did not reset progress");
  auto timeout = vacuum();
  timeout.add_target(TargetConfig::rune("small", {5, 0, 2.6}, false));
  fire_at_rune(timeout, 0);
  timeout.step(2600);
  check(timeout.snapshot().targets[0].completed_groups == 0, "2.5 second deadline not enforced");
  auto big = vacuum();
  big.add_target(TargetConfig::rune("big", {5, 0, 2.6}, true));
  for (int i = 0; i < 10; ++i) {
    big.step(60);
    fire_at_rune(big, i);
  }
  check(big.snapshot().targets[0].activation == "activated",
        "five pairs did not activate big rune");
  check(big.snapshot().targets[0].accepted_hits == 10, "big rune second-hit window failed");
  auto singles = vacuum();
  singles.add_target(TargetConfig::rune("big", {5, 0, 2.6}, true));
  for (int i = 0; i < 5; ++i) {
    fire_at_rune(singles, i);
    singles.step(1001);
  }
  check(singles.snapshot().targets[0].activation == "activated",
        "one per group should activate big rune");
}
void determinism_and_validation()
{
  auto a = vacuum();
  auto c = car();
  c.translation[1].kind = MotionKind::random;
  a.add_target(c);
  World b = a;
  a.step(1000);
  for (int i = 0; i < 100; ++i) b.step(10);
  near(a.snapshot().targets[0].position.y, b.snapshot().targets[0].position.y, 0);
  rejects([&] { a.add_target(car("late")); });
  auto v = vacuum();
  v.add_target(c);
  rejects([&] { v.add_target(c); });
  rejects([&] { v.launch({"bad", {0, 0, -1}, {20, 0, 0}, 1}); });
  rejects([&] { v.stop_outpost("car"); });
  rejects([&] { v.activate_rune("missing"); });
  c.radius = 0;
  rejects([&] { c.validate(); });
  c = car();
  c.position.x = std::numeric_limits<double>::quiet_NaN();
  rejects([&] { c.validate(); });
  WorldConfig bad;
  bad.timestep = .01;
  rejects([&] { World w(bad); });
}
}  // namespace
int main()
{
  try {
    motion_tests();
    geometry_tests();
    rule_motion_tests();
    collision_tests();
    rune_tests();
    determinism_and_validation();
    std::cout << checks << " checks passed\n";
  } catch (const std::exception &e) {
    std::cerr << "FAIL after " << checks << " checks: " << e.what() << "\n";
    return 1;
  }
}
