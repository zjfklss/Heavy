#pragma once
#include <string>
namespace tools::jlcv_show
{
class Channel;
}
struct SimulationOptions
{
  std::string config, host = "127.0.0.1", project, preset;
  int port = 17001;
  bool show = false, once = false, launch_ui = true, no_fire = false;
};
int run_hero_simulation(const SimulationOptions & options, tools::jlcv_show::Channel * display = nullptr);
