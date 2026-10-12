#ifndef PROJECTILE_MOTION__ITERATIVE_PROJECTILE_TOOL_HPP_
#define PROJECTILE_MOTION__ITERATIVE_PROJECTILE_TOOL_HPP_

#include <string>
#include <functional>

namespace projectile_motion
{

// Generic iterative solver for projectile inverse kinematics
class IterativeProjectileTool
{
  // Forward motion function type:
  // given_angle: launch angle (rad), given_x: horizontal distance (m)
  // h: output landing height (m), t: output flight time (s)
  typedef std::function<void(double given_angle, double given_x, double & h, double & t)>
    ForwardMotionFunc;

public:
  IterativeProjectileTool() {}

  void set_max_iter(int max_iter) { max_iter_ = max_iter; }
  void set_forward_motion(ForwardMotionFunc forward_motion) { forward_motion_func_ = forward_motion; }

  bool solve(double target_x, double target_h, double & angle);
  // 产品锁定：ITL 原 solve() 从 atan2(h,x) 起步会落到平射支。
  // 这里用真空高抛根 (v²+√Δ)/(g x) 作 aimed_h 初值，迭代循环仍是 ITL 原文。
  bool solve_high(
    double target_x, double target_h, double speed, double gravity, double & angle);
  std::string error_message() { return error_message_; }

private:
  int max_iter_{20};
  ForwardMotionFunc forward_motion_func_;
  std::string error_message_;
};

}  // namespace projectile_motion

#endif  // PROJECTILE_MOTION__ITERATIVE_PROJECTILE_TOOL_HPP_
