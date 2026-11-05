#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "io/mode.hpp"
#include "solver.hpp"
#include "target.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  bool state();
  std::string state_str();
  Target get_target();
  void set_enemy_color(Color color) { enemy_color_ = color; }

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t, double yaw, bool use_enemy_color = true,
    io::Mode mode = io::Mode::auto_aim);

private:
  const static int ENEMY_NUM = 8;
  Solver & solver_;
  Color enemy_color_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  io::Mode last_mode_ = io::Mode::idle;
  Target targets_[ENEMY_NUM];
  int last_target_name_ = -1; // -1 for none

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t, io::Mode mode);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_targets(std::list<Armor> & armors, std::chrono::steady_clock::time_point t, double yaw);

  void clear_targets();
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
