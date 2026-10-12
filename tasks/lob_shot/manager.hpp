#ifndef LOB_SHOT__MANAGER_HPP_
#define LOB_SHOT__MANAGER_HPP_

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "command.hpp"

namespace lob_shot
{

enum class State
{
  IDLE,
  INHIBITED,
  WAITING_POSE,
  YAW_AIMING,
  YAW_CONVERGING,
  PITCH_AIMING,
  PITCH_CONVERGING,
  READY,
};

struct Config
{
  double target_x = 25.603;
  double target_y = 8.000;
  double target_z = 1.602;
  double base_x = 3.0;
  double base_y = 1.5;
  double base_z = 0.0;
  double base_yaw = 0.0;  // 底盘在 map 下朝向；输出 yaw = 地图角 - base_yaw
  double muzzle_dx = 0.0;
  double muzzle_dy = 0.0;
  double muzzle_dz = 0.4;
  double bullet_speed = 16.5;
  double friction = 0.0;
  double pitch_max_deg = 42.0;
  double yaw_tol = 0.02;
  double pitch_tol = 0.02;
  double aim_timeout_s = 5.0;
  double pose_timeout_s = 5.0;
  double pitch_sign = 1.0;
  double yaw_sign = 1.0;
  bool open_loop = true;
  // 实车默认应全 false；dry_run yaml 再打开
  bool module_unlocked = false;
  bool module_docked = false;
  bool module_assembling = false;
  double windmill_x = 14.5;
  double windmill_y = 8.0;
  double windmill_radius = 1.7;
  double windmill_z_min = 0.2;
  double windmill_z_max = 2.8;
};

inline double wrap_pi(double a)
{
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

class Manager
{
public:
  explicit Manager(Config cfg);

  // 边沿：内部置 pending，消费后清除（对齐 Heavy triggered_）
  void request_trigger();
  void request_cancel();
  void set_fire(bool on);
  void set_gate(bool unlocked, bool docked, bool assembling);
  void set_feedback(double yaw, double pitch);
  void set_bullet_speed(double v);

  Command tick(double dt_s = 0.02);

  State state() const { return state_; }
  const std::string & status() const { return status_; }
  double last_pitch_deg() const { return last_pitch_deg_; }
  double last_yaw_deg() const { return last_yaw_deg_; }
  double last_high_deg() const { return last_high_deg_; }
  double last_dist() const { return last_dist_; }
  bool traj_blocked() const { return traj_blocked_; }
  bool fire_latched() const { return fire_; }
  const std::vector<std::pair<double, double>> & traj_xz() const { return traj_xz_; }

private:
  bool gate_ok() const;
  void set_state(State s, const std::string & name);
  void clear_aim_request();  // 清 trigger pending + fire
  void solve_yaw();
  bool solve_pitch_low_only();
  bool hits_windmill(double mx, double my, double mz) const;
  void build_traj(double pitch);

  Config cfg_;
  State state_ = State::IDLE;
  std::string status_ = "IDLE";
  bool trigger_pending_ = false;
  bool fire_ = false;
  double t_in_state_ = 0;
  double fb_yaw_ = 0;
  double fb_pitch_ = 0;
  double target_yaw_ = 0;    // 相对底盘
  double target_yaw_map_ = 0;
  double target_pitch_ = 0;
  double last_pitch_deg_ = 0;
  double last_yaw_deg_ = 0;
  double last_high_deg_ = 0;
  double last_dist_ = 0;
  bool traj_blocked_ = false;
  std::vector<std::pair<double, double>> traj_xz_;
};

}  // namespace lob_shot

#endif
