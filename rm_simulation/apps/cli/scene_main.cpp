#include "rm_sim/scenario.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <set>

using namespace rm_sim;
using namespace rm_sim::scenario;
using nlohmann::json;

int main(int argc, char **argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: rm_sim scenario.json output.json\nStandalone target world; metres, "
                 "seconds, radians.\n";
    return 0;
  }
  try {
    if (argc != 3) throw std::invalid_argument("Usage: rm_sim scenario.json output.json");
    std::ifstream input(argv[1]);
    if (!input) throw std::runtime_error("Cannot open scenario");
    const auto scenario = json::parse(input);
    keys(scenario, {"schema_version", "name", "duration", "sample_period", "world", "targets",
                    "shots", "events"});
    if (scenario.at("schema_version") != 1)
      throw std::invalid_argument("Unsupported schema_version");
    WorldConfig cfg;
    if (scenario.contains("world")) {
      const auto &w = scenario["world"];
      keys(w, {"timestep", "gravity", "ground_z", "minimum_normal_speed", "detection_interval"});
      cfg.timestep = w.value("timestep", cfg.timestep);
      if (w.contains("gravity")) cfg.gravity = vec(w["gravity"]);
      cfg.ground_z = w.value("ground_z", cfg.ground_z);
      cfg.minimum_normal_speed = w.value("minimum_normal_speed", cfg.minimum_normal_speed);
      cfg.detection_interval = w.value("detection_interval", cfg.detection_interval);
    }
    World world(cfg);
    const auto duration = ticks(scenario.at("duration").get<double>(), cfg.timestep);
    const auto stride = ticks(scenario.value("sample_period", .02), cfg.timestep);
    if (stride == 0 || duration == 0 || duration > 10000000 || duration / stride > 30000)
      throw std::invalid_argument("Invalid duration/sample period or excessive sample count");
    if (!scenario.at("targets").is_array() || scenario["targets"].empty() ||
        scenario["targets"].size() > 64)
      throw std::invalid_argument("Need 1..64 targets");
    for (const auto &t : scenario["targets"]) world.add_target(target(t));
    struct TimedShot
    {
      uint64_t tick;
      Shot shot;
    };
    std::vector<TimedShot> shots;
    std::set<std::string> shot_ids;
    for (const auto &s : scenario.value("shots", json::array())) {
      keys(s, {"time", "id", "origin", "velocity", "lifetime"});
      Shot shot{s.at("id").get<std::string>(), vec(s.at("origin")), vec(s.at("velocity")),
                s.value("lifetime", 3.0)};
      if (!shot_ids.insert(shot.id).second) throw std::invalid_argument("Duplicate shot id");
      const auto tick = ticks(s.at("time").get<double>(), cfg.timestep);
      if (tick >= duration) throw std::invalid_argument("Shot time must precede end of simulation");
      shots.push_back({tick, shot});
    }
    std::stable_sort(shots.begin(), shots.end(), [](auto &a, auto &b) { return a.tick < b.tick; });
    struct Event
    {
      uint64_t tick;
      std::string type, id;
    };
    std::vector<Event> events;
    for (const auto &e : scenario.value("events", json::array())) {
      keys(e, {"time", "type", "target_id"});
      Event event{ticks(e.at("time").get<double>(), cfg.timestep), e.at("type"), e.at("target_id")};
      if (event.tick >= duration || (event.type != "activate_rune" && event.type != "stop_outpost"))
        throw std::invalid_argument("Invalid event type/time");
      events.push_back(event);
    }
    std::stable_sort(events.begin(), events.end(),
                     [](auto &a, auto &b) { return a.tick < b.tick; });
    json output = {{"schema_version", 1},
                   {"engine", "rm_sim_world/1"},
                   {"scenario", scenario},
                   {"rules",
                    {{"armor", "RMU 2026 Robot Specification V2.0.0, pp.67-70"},
                     {"props", "RMUC 2027 Rule Manual V1.0.0, pp.54-56,95-101"}}},
                   {"units", "m,s,rad; x-forward y-left z-up; surface normals point outward"},
                   {"model",
                    "point projectiles; gravity only; rectangular armor envelope; no chassis "
                    "collisions or match HP"},
                   {"frames", json::array()},
                   {"hits", json::array()}};
    size_t shot_index = 0, event_index = 0;
    for (uint64_t i = 0; i <= duration; ++i) {
      while (event_index < events.size() && events[event_index].tick == i) {
        const auto &e = events[event_index++];
        if (e.type == "activate_rune")
          world.activate_rune(e.id);
        else
          world.stop_outpost(e.id);
      }
      while (shot_index < shots.size() && shots[shot_index].tick == i)
        world.launch(shots[shot_index++].shot);
      if (i % stride == 0 || i == duration) output["frames"].push_back(frame(world.snapshot()));
      if (i < duration) world.step();
    }
    int accepted = 0;
    for (const auto &h : world.hits()) {
      accepted += h.effective;
      output["hits"].push_back({{"time", h.time},
                                {"shot_id", h.shot_id},
                                {"target_id", h.target_id},
                                {"plate_id", h.plate_id},
                                {"point", js(h.point)},
                                {"plate_point", h.plate_point ? js(*h.plate_point) : json(nullptr)},
                                {"normal_speed", h.normal_speed},
                                {"effective", h.effective},
                                {"reason", h.reason},
                                {"ring", h.ring}});
    }
    output["summary"] = {
      {"shots", shots.size()}, {"contacts", world.hits().size()}, {"effective_hits", accepted}};
    // Serialize only after successful validation/run; a failed simulation does not leave partial output.
    std::ofstream out(argv[2]);
    if (!out) throw std::runtime_error("Cannot open output");
    out << output.dump();
    if (!out) throw std::runtime_error("Cannot write output");
    std::cout << output["summary"].dump() << "\n";
  } catch (const std::exception &e) {
    std::cerr << "[rm_sim] " << e.what() << "\n";
    return 1;
  }
}
