#include "launcher.hpp"

#include <arpa/inet.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <thread>
#include <vector>

extern char ** environ;

namespace hero
{
namespace
{
namespace fs = std::filesystem;

fs::path simulator_project(const std::string & project)
{
  if (!project.empty()) return fs::absolute(project);
  // 优先用仓库内置的快照(Heavy/rm_simulation)，没有再退回同级目录，和 sp_jlcv_aimer 的行为一致
  const auto bundled = fs::path(HERO_SOURCE_ROOT) / "rm_simulation";
  if (fs::is_directory(bundled)) return bundled;
  return fs::path(HERO_SOURCE_ROOT).parent_path() / "rm_simulation";
}
}  // namespace

fs::path select_profile(const std::string & config, bool simulation)
{
  const fs::path root(HERO_SOURCE_ROOT);
  fs::path selected;
  if (config.empty()) {
    selected = root / "configs";
    if (simulation) selected /= "simulation";
    selected /= "hero.yaml";
  } else {
    selected = config;
    if (selected.is_relative() && !fs::is_regular_file(selected)) selected = root / selected;
  }
  if (!fs::is_regular_file(selected)) throw std::invalid_argument("Missing hero profile: " + selected.string());
  return fs::absolute(selected).lexically_normal();
}

fs::path simulator_preset(const std::string & preset, const std::string & project)
{
  const auto presets = simulator_project(project) / "configs/scenarios/sensors";
  if (preset.empty()) return presets / "hero.json";
  if (preset.find('/') == std::string::npos && fs::path(preset).extension().empty())
    return presets / (preset + ".json");
  return fs::absolute(preset);
}

SimulationLauncher::SimulationLauncher(
  const std::string & host, int port, const std::string & project, const std::string & preset)
{
  in_addr address{};
  if (port < 1 || port > 65535 || inet_pton(AF_INET, host.c_str(), &address) != 1)
    throw std::invalid_argument("Invalid simulation IPv4/port");
  const auto root = simulator_project(project);
  const auto script = root / "start_simulation.sh";
  const auto source = simulator_preset(preset, project);
  if (!fs::is_regular_file(script) || access(script.c_str(), X_OK) != 0)
    throw std::invalid_argument("Missing executable simulator launcher: " + script.string());
  std::ifstream input(source);
  if (!input)
    throw std::invalid_argument(
      "Missing simulator preset: " + source.string() + "; choose one with --sim-preset");
  auto config = nlohmann::json::parse(input);
  const auto previous = config.value("algorithm", nlohmann::json::object());
  config["algorithm"] = {
    {"kind", "external"},
    {"adapter", "generic"},
    {"host", host == "0.0.0.0" ? "127.0.0.1" : host},
    {"port", port},
    {"connect_timeout", previous.value("connect_timeout", 5.0)},
    {"step_timeout", previous.value("step_timeout", 10.0)}};
  config["sensor"]["enabled"] = true;
  config["sensor"]["output"] = "image";
  config["actuator"]["feedback_enabled"] = true;

  const auto artifacts_override = std::getenv("RM_SIM_ARTIFACT_DIR");
  const auto artifacts = artifacts_override && *artifacts_override ? fs::absolute(artifacts_override)
                                                                   : root.parent_path() / "rm_simulation-artifacts";
  const auto runs = artifacts / "runs/hero";
  fs::create_directories(runs);
  auto pattern = (runs / "simulation-hero-XXXXXX").string();
  std::vector<char> directory(pattern.begin(), pattern.end());
  directory.push_back('\0');
  if (!mkdtemp(directory.data())) throw std::runtime_error("Cannot create simulation run directory");
  config_path_ = fs::path(directory.data()) / "configuration.json";
  std::ofstream output(config_path_);
  output << config.dump(2) << '\n';
  output.close();
  if (!output) throw std::runtime_error("Cannot write simulator configuration");

  std::string executable = script.string(), argument = config_path_.string();
  char option[] = "--config";
  char * argv[] = {executable.data(), option, argument.data(), nullptr};
  posix_spawnattr_t attributes;
  int error = posix_spawnattr_init(&attributes);
  if (error) throw std::runtime_error("Cannot initialize simulator process: " + std::string(strerror(error)));
  // posix_spawn remains safe when the diagnostic dashboard already runs another thread.
  error = posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
  if (!error) error = posix_spawnattr_setpgroup(&attributes, 0);
  if (!error) error = posix_spawn(&process_, executable.c_str(), nullptr, &attributes, argv, environ);
  posix_spawnattr_destroy(&attributes);
  if (error) throw std::runtime_error("Cannot start simulator: " + std::string(strerror(error)));
  std::cout << "[Hero simulation] opening " << source << "; run config " << config_path_ << std::endl;
}

void SimulationLauncher::check()
{
  if (finished_) return;
  int status = 0;
  const auto result = waitpid(process_, &status, WNOHANG);
  if (result == 0 || (result < 0 && errno == EINTR)) return;
  if (result < 0) throw std::runtime_error("Cannot read simulator process status");
  finished_ = true;
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    kill(-process_, SIGTERM);
    throw std::runtime_error("Simulator build/UI exited unsuccessfully; see its output above");
  }
}

SimulationLauncher::~SimulationLauncher()
{
  if (process_ < 0 || finished_) return;
  kill(-process_, SIGTERM);
  for (int i = 0; i < 100; ++i) {
    const auto result = waitpid(process_, nullptr, WNOHANG);
    if (result == process_ || (result < 0 && errno == ECHILD)) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  kill(-process_, SIGKILL);
  while (waitpid(process_, nullptr, 0) < 0 && errno == EINTR) {
  }
}
}  // namespace hero
