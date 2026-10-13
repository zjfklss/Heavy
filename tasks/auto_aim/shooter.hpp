#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
// 开火判定：相邻两拍指令 yaw 差 < 2×容差，且云台 yaw 已跟到 < 容差，才置 shoot。
// 容差按距离分档（> judge_distance 用 second_tolerance）；钉头时改看 Aimer 的开火窗。auto_fire=false 永不开火。
class Shooter
{
public:
  Shooter(const std::string & config_path);

  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  bool auto_fire_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP