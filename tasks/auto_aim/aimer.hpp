#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

  // 重装钉头：云台 yaw 钉车中心，窗口内开火（抄自 jlauto-aim/hero processAssessment2）
  bool nail_mode() const { return nail_mode_; }
  bool nail_latched() const { return nail_latched_; }

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;

  bool nail_mode_ = false;
  double nail_spin_on_ = 1.0;   // rad/s，进入钉头
  double nail_spin_off_ = 0.7;  // rad/s，退出钉头（滞回）
  double nail_fire_window_ = 4.0 / 57.3;
  bool nail_latched_ = false;

  AimPoint choose_aim_point(const Target & target);
  void update_nail_latch(double abs_w);
  io::Command aim_track(
    Target target, std::chrono::steady_clock::time_point future, double bullet_speed);
  io::Command aim_nail(
    Target target, std::chrono::steady_clock::time_point future, double bullet_speed);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
