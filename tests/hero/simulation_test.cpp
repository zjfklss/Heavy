#include <gtest/gtest.h>
#include <sys/stat.h>

#include <atomic>
#include <cmath>
#include <fstream>
#include <thread>

#include "io/simulation/bridge.hpp"
#include "src/simulation/launcher.hpp"
#include "src/simulation/runner.hpp"
#include "tools/math_tools.hpp"

namespace
{
using nlohmann::json;
using namespace rm_sim::adapters;
const std::string root = HERO_TEST_ROOT;

json rotation_z(double angle)
{
  // Simulator rotations are column lists.
  return {{std::cos(angle), std::sin(angle), 0}, {-std::sin(angle), std::cos(angle), 0}, {0, 0, 1}};
}
json hello(double speed, double base_yaw = 0)
{
  const json identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  return {
    {"protocol", wire_version},
    {"type", "hello"},
    {"session_id", "test"},
    {"adapter", "generic"},
    {"units", {{"pitch_up_sign", 1}}},
    {"configuration",
     {{"realtime", false},
      {"actuator",
       {{"yaw_offset", {0, 0, 0}},
        {"pitch_offset", {0, 0, 0}},
        {"muzzle_mount", {{"translation", {0, 0, 0}}, {"rotation", identity}}},
        {"base_pose", {{"rotation", rotation_z(base_yaw)}}},
        {"camera_mount", {{"translation", {0.1, 0, 0.05}}, {"rotation", {{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}}}},
        {"nominal_muzzle_speed", speed},
        {"fire_interval", 0.1},
        {"feedback_frequency", 1000.0},
        {"feedback_latency", 0.0}}},
      {"camera",
       {{"calibration", {{"fx", 2400.0}, {"fy", 2400.0}, {"cx", 720.0}, {"cy", 540.0}, {"width", 1440}, {"height", 1080}}},
        {"latency", 0.0},
        {"enemy_color", "red"}}}}}};
}
Connection connect(int port)
{
  for (int attempt = 0; attempt < 200; ++attempt) {
    try {
      return Connection::connect("127.0.0.1", port, deadline_after(1000));
    } catch (const std::exception &) {
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
  }
  throw std::runtime_error("Cannot connect to hero simulation server");
}
void handshake(Connection & client, const json & message)
{
  client.send({message, {}}, deadline_after(2000));
  const auto ack = client.receive(deadline_after(30000));
  ASSERT_EQ(ack.header.at("type"), "hello_ack");
}
Message step(uint64_t tick, double yaw, double pitch, bool image, int width = 1440, int height = 1080)
{
  const double dt = 0.01, time = tick * dt;
  Message message;
  message.header = {{"protocol", wire_version}, {"type", "step"}, {"session_id", "test"}, {"sequence", tick},
                    {"tick", tick}, {"time", time}, {"timestep", dt}, {"images", json::array()},
                    {"feedback", json::array({{{"sequence", tick}, {"capture_tick", tick}, {"ready_tick", tick},
                                               {"capture_time", time}, {"ready_time", time}, {"yaw", yaw},
                                               {"pitch", pitch}, {"yaw_velocity", 0.0}, {"pitch_velocity", 0.0}}})}};
  if (image) {
    const size_t bytes = size_t(width) * height * 3;
    message.header["images"].push_back(
      {{"sequence", tick}, {"sensor_id", "camera"}, {"calibration_id", "test"}, {"capture_tick", tick},
       {"ready_tick", tick}, {"capture_time", time}, {"ready_time", time}, {"width", width}, {"height", height},
       {"stride", width * 3}, {"encoding", "bgr8"}, {"offset", 0}, {"bytes", bytes}});
    message.payload.assign(bytes, 0);
  }
  return message;
}
}  // namespace

TEST(HeroSimulation, AnglesArePitchUpPositive)
{
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const auto up = io::simulation::to_world(identity, 0, 0.3);
  EXPECT_NEAR(up.pitch, 0.3, 1e-12);
  // The hero gimbal pitch shown in logs is -eulers(R, 2, 1, 0)[1].
  const Eigen::Vector3d ypr = tools::eulers(hero::attitude(0.2, 0.1).toRotationMatrix(), 2, 1, 0);
  EXPECT_NEAR(ypr[0], 0.2, 1e-12);
  EXPECT_NEAR(-ypr[1], 0.1, 1e-12);
  const Eigen::Matrix3d base = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto world = io::simulation::to_world(base, 0.2, 0.1);
  EXPECT_NEAR(world.yaw, 0.2 + M_PI / 2, 1e-12);
  EXPECT_NEAR(world.pitch, 0.1, 1e-12);
  const auto local = io::simulation::to_simulator(base, world.yaw, world.pitch);
  EXPECT_NEAR(local.yaw, 0.2, 1e-12);
  EXPECT_NEAR(local.pitch, 0.1, 1e-12);
}

TEST(HeroSimulation, BridgeRoundTripsFeedbackAndCommandsThroughRotatedBase)
{
  ServerOptions options;
  options.adapter = io::simulation::adapter;
  options.port = 17931;
  options.accept_timeout_ms = 5000;
  io::simulation::Bridge bridge(options);
  json reply;
  std::thread client([&] {
    auto connection = connect(options.port);
    handshake(connection, hello(15.8, M_PI / 2));
    connection.send(step(0, 0.2, 0.1, true, 64, 48), deadline_after(2000));
    reply = connection.receive(deadline_after(5000)).header;
    connection.send({{{"protocol", wire_version}, {"type", "finish"}, {"session_id", "test"}, {"tick", 1}}, {}},
                    deadline_after(2000));
    connection.receive(deadline_after(5000));
  });
  bridge.accept(root + "/configs/simulation/hero.yaml");
  EXPECT_EQ(bridge.enemy_color(), 0);
  EXPECT_DOUBLE_EQ(bridge.bullet_speed(), 15.8);
  const auto session = YAML::LoadFile(bridge.config_path());
  EXPECT_EQ(session["enemy_color"].as<std::string>(), "red");
  EXPECT_DOUBLE_EQ(session["serial"]["bullet_speed"].as<double>(), 15.8);
  EXPECT_EQ(session["camera_matrix"].as<std::vector<double>>()[0], 2400);
  auto next = bridge.next();
  ASSERT_TRUE(next);
  ASSERT_EQ(next->frames.size(), 1u);
  ASSERT_EQ(next->feedback.size(), 1u);
  EXPECT_NEAR(next->feedback[0].yaw, 0.2 + M_PI / 2, 1e-6);  // hero::Feedback stores wire floats
  EXPECT_NEAR(next->feedback[0].pitch, 0.1, 1e-6);  // hero::Feedback stores wire floats
  const Eigen::Vector3d ypr = tools::eulers(next->frames[0].attitude.toRotationMatrix(), 2, 1, 0);
  EXPECT_NEAR(ypr[0], 0.2 + M_PI / 2, 1e-6);  // hero::Feedback stores wire floats
  EXPECT_NEAR(-ypr[1], 0.1, 1e-6);  // hero::Feedback stores wire floats
  // A hero command in world angles comes back to the simulator in its local base frame.
  bridge.reply(io::Command{true, true, ypr[0] + 0.05, -ypr[1] + 0.02}, json::object());
  EXPECT_FALSE(bridge.next());
  client.join();
  ASSERT_EQ(reply.at("commands").size(), 1u);
  const auto & command = reply["commands"][0];
  EXPECT_TRUE(command.at("control").get<bool>());
  EXPECT_TRUE(command.at("fire").get<bool>());
  EXPECT_NEAR(command.at("yaw").get<double>(), 0.25, 1e-6);  // hero::Feedback stores wire floats
  EXPECT_NEAR(command.at("pitch").get<double>(), 0.12, 1e-6);  // hero::Feedback stores wire floats
}

TEST(HeroSimulation, RejectsSeventeenMillimetreMuzzleSpeed)
{
  ServerOptions options;
  options.adapter = io::simulation::adapter;
  options.port = 17932;
  options.accept_timeout_ms = 5000;
  io::simulation::Bridge bridge(options);
  std::thread client([&] {
    auto connection = connect(options.port);
    connection.send({hello(23), {}}, deadline_after(2000));
    try {
      connection.receive(deadline_after(5000));
    } catch (const std::exception &) {
    }
  });
  EXPECT_THROW(bridge.accept(root + "/configs/simulation/hero.yaml"), std::invalid_argument);
  client.join();
}

TEST(HeroSimulation, RunnerProcessesSessionWithoutHardware)
{
  SimulationOptions options;
  options.config = (hero::select_profile("", true)).string();
  options.port = 17933;
  options.launch_ui = false;
  options.once = true;
  std::atomic<int> result{-1};
  std::thread server([&] { result = run_hero_simulation(options); });
  std::vector<json> replies;
  {
    auto connection = connect(options.port);
    handshake(connection, hello(15.8));
    for (uint64_t tick = 0; tick < 4; ++tick) {
      connection.send(step(tick, 0.0, 0.0, true), deadline_after(2000));
      replies.push_back(connection.receive(deadline_after(60000)).header);
    }
    connection.send({{{"protocol", wire_version}, {"type", "finish"}, {"session_id", "test"}, {"tick", 4}}, {}},
                    deadline_after(2000));
    connection.receive(deadline_after(5000));
  }
  server.join();
  EXPECT_EQ(result, 0);
  ASSERT_EQ(replies.size(), 4u);
  EXPECT_EQ(replies.back().at("telemetry").at("frames_processed"), 4);
  EXPECT_EQ(replies.back().at("telemetry").at("state"), "running");
  for (const auto & reply : replies) {
    ASSERT_EQ(reply.at("commands").size(), 1u);
    EXPECT_FALSE(reply["commands"][0].at("fire").get<bool>());  // blank frames: no target, no fire
  }
}

TEST(HeroSimulation, ProfileAndPresetSelection)
{
  EXPECT_EQ(hero::select_profile("", false), std::filesystem::path(root) / "configs/hero.yaml");
  EXPECT_EQ(hero::select_profile("", true), std::filesystem::path(root) / "configs/simulation/hero.yaml");
  EXPECT_EQ(hero::select_profile("configs/hero.yaml", true), std::filesystem::path(root) / "configs/hero.yaml");
  EXPECT_THROW(hero::select_profile("configs/missing.yaml", false), std::invalid_argument);
  const std::string project = "/tmp/sim";
  EXPECT_EQ(hero::simulator_preset("", project), "/tmp/sim/configs/scenarios/sensors/hero.json");
  EXPECT_EQ(hero::simulator_preset("infantry4", project), "/tmp/sim/configs/scenarios/sensors/infantry4.json");
  EXPECT_EQ(hero::simulator_preset("/x/y.json", project), "/x/y.json");
}

TEST(HeroSimulation, LauncherWritesGenericAdapterRunConfig)
{
  char directory[] = "/tmp/hero-sim-project-XXXXXX";
  ASSERT_TRUE(mkdtemp(directory));
  const std::filesystem::path project(directory);
  std::filesystem::create_directories(project / "configs/scenarios/sensors");
  std::ofstream(project / "configs/scenarios/sensors/infantry4.json")
    << R"({"algorithm":{"step_timeout":7},"sensor":{},"actuator":{}})";
  const auto script = project / "start_simulation.sh";
  std::ofstream(script) << "#!/bin/sh\necho \"$2\" > \"$(dirname \"$0\")/started\"\n";
  ::chmod(script.c_str(), 0755);
  setenv("RM_SIM_ARTIFACT_DIR", (project / "artifacts").c_str(), 1);
  EXPECT_THROW(hero::SimulationLauncher("127.0.0.1", 17001, project.string()), std::invalid_argument);
  std::filesystem::path run;
  {
    hero::SimulationLauncher launcher("127.0.0.1", 17934, project.string(), "infantry4");
    run = launcher.config_path();
    for (int i = 0; i < 200 && !std::filesystem::exists(project / "started"); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    launcher.check();
  }
  unsetenv("RM_SIM_ARTIFACT_DIR");
  std::ifstream input(run);
  const auto config = json::parse(input);
  EXPECT_EQ(config["algorithm"]["adapter"], "generic");
  EXPECT_EQ(config["algorithm"]["port"], 17934);
  EXPECT_EQ(config["algorithm"]["step_timeout"], 7);
  EXPECT_TRUE(config["sensor"]["enabled"].get<bool>());
  EXPECT_EQ(config["sensor"]["output"], "image");
  EXPECT_TRUE(config["actuator"]["feedback_enabled"].get<bool>());
  std::string started;
  std::getline(std::ifstream(project / "started"), started);
  EXPECT_EQ(started, run.string());
  std::filesystem::remove_all(project);
}
