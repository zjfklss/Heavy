#pragma once

#include <sys/types.h>

#include <filesystem>
#include <string>

namespace hero
{
// An explicit YAML keeps its original meaning; otherwise choose configs/hero.yaml or its simulation copy.
std::filesystem::path select_profile(const std::string & config, bool simulation);

// Simulator preset JSON: empty selects <project>/configs/scenarios/sensors/hero.json; a bare name such as
// "infantry4" selects that preset; anything else is a path.
std::filesystem::path simulator_preset(const std::string & preset, const std::string & project = "");

// Owns the simulator UI and its backend as a separate process group.
class SimulationLauncher
{
public:
  SimulationLauncher(
    const std::string & host, int port, const std::string & project = "", const std::string & preset = "");
  ~SimulationLauncher();
  SimulationLauncher(const SimulationLauncher &) = delete;
  SimulationLauncher & operator=(const SimulationLauncher &) = delete;

  void check();
  const std::filesystem::path & config_path() const { return config_path_; }

private:
  std::filesystem::path config_path_;
  pid_t process_ = -1;
  bool finished_ = false;
};
}  // namespace hero
