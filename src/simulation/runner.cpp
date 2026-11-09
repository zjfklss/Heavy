#include "runner.hpp"

#include <csignal>
#include <iostream>
#include <memory>

#include "io/simulation/bridge.hpp"
#include "launcher.hpp"
#include "src/hero_show.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_outpost/shooter.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

namespace
{
volatile std::sig_atomic_t interrupted = 0;
}

int run_hero_simulation(const SimulationOptions & options, tools::jlcv_show::Channel * display)
{
  // The hero algorithms run here, in the independent vision process, never inside the simulator.
  std::signal(SIGINT, [](int) { interrupted = 1; });
  std::signal(SIGTERM, [](int) { interrupted = 1; });
  const auto stopping = [&] { return interrupted || (display && display->stopping()); };
  {
    // Validates the user YAML before listening; per-session files reuse every model and threshold.
    auto base = io::simulation::load_base_config(options.config);
    (void)base;
  }
  rm_sim::adapters::ServerOptions server;
  server.host = options.host;
  server.port = options.port;
  server.adapter = io::simulation::adapter;
  server.peer_version = "heavy-hero/simulation-1";
  server.accept_timeout_ms = 1000;
  server.idle_timeout_ms = 10000;
  std::unique_ptr<hero::SimulationLauncher> simulator;
  if (options.launch_ui)
    simulator =
      std::make_unique<hero::SimulationLauncher>(options.host, options.port, options.project, options.preset);
  if (display) {
    display->reset("仿真 / 英雄");
    display->status("等待连接");
  }
  std::unique_ptr<io::simulation::Bridge> connection;
  std::unique_ptr<auto_aim::multithread::MultiThreadDetector> detector;
  while (!stopping()) {
    if (simulator) simulator->check();
    if (!connection) {
      connection = std::make_unique<io::simulation::Bridge>(server);
      std::cout << "[Hero simulation] listening " << options.host << ":" << options.port
                << "; no camera or serial opened" << std::endl;
    }
    bool accepted = false;
    try {
      auto & bridge = *connection;
      bridge.accept(options.config);
      accepted = true;
      if (display) {
        display->reset("仿真 / 英雄");
        display->status("已连接");
      }
      const auto path = bridge.config_path();
      // Compile the OpenVINO model once; the detector reads no calibration that sessions override.
      if (!detector) detector = std::make_unique<auto_aim::multithread::MultiThreadDetector>(path, false);
      auto_aim::Solver solver(path);
      auto_aim::Tracker tracker(path, solver);
      auto_outpost::Shooter shooter(path);
      tracker.set_enemy_color(bridge.enemy_color() == 0 ? auto_aim::red : auto_aim::blue);
      std::cout << "[Hero simulation] connected; BGR + timestamped gimbal feedback -> hero -> commands"
                << std::endl;
      uint64_t processed = 0, armors_count = 0, targets_count = 0;
      io::Command last{false, false, 0, 0};
      while (!stopping()) {
        auto step = bridge.next();
        if (!step) break;
        if (display)
          for (const auto & f : step->feedback) display->feedback(f.time, f.yaw, f.pitch);
        std::optional<io::Command> command;
        for (auto & frame : step->frames) {
          solver.set_R_gimbal2world(frame.attitude);
          const Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
          detector->push(frame.bgr, frame.stamp);
          auto [image, armors, stamp] = detector->debug_pop();
          armors_count = armors.size();
          auto targets = tracker.track(armors, stamp, ypr[0], true, io::Mode::auto_aim);
          targets_count = targets.size();
          // Live sessions share the local steady-clock epoch; batch sessions keep logical time.
          last = shooter.shoot(targets, stamp, bridge.bullet_speed(), bridge.realtime(), ypr);
          if (options.no_fire) last.shoot = false;
          command = last;
          ++processed;
          if (options.show) {
            for (const auto & armor : armors) tools::draw_points(image, armor.points, {0, 255, 255});
            publish_hero_estimate(display, targets, shooter.debug_aim_point_, image);
          }
        }
        bridge.reply(
          command, {{"state", processed ? "running" : "waiting_sensors"},
                    {"frames_processed", processed},
                    {"armors", armors_count},
                    {"targets", targets_count},
                    {"tracker", tracker.state_str()}});
        if (display && command)
          display->command(step->time, command->yaw, command->pitch, command->control, command->shoot);
      }
      std::cout << "[Hero simulation] finished frames=" << processed << "; waiting for next UI run" << std::endl;
      if (options.once) return 0;
      if (display) display->status("等待下次连接");
      connection.reset();
    } catch (const std::exception & error) {
      if (stopping()) return 0;
      const bool waiting = !accepted && std::string(error.what()) == "External adapter timeout";
      if (waiting) continue;
      std::cerr << "[Hero simulation] session ended: " << error.what() << std::endl;
      if (options.once) return 1;
      connection.reset();
    }
  }
  return 0;
}
