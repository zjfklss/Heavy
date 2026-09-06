#include "projectile_motion/iterative_projectile_tool.hpp"

#include <cmath>

namespace projectile_motion
{

bool IterativeProjectileTool::solve(double target_x, double target_h, double & angle)
{
  double aimed_h = target_h;
  double dh = 0;
  double tmp_angle = 0;
  double h = 0;
  double t = 0;

  for (int i = 0; i < max_iter_; i++) {
    tmp_angle = atan2(aimed_h, target_x);
    if (tmp_angle > 80 * M_PI / 180 || tmp_angle < -80 * M_PI / 180) {
      error_message_ = "iterative angle out of range (-80, 80) deg";
      return false;
    }
    forward_motion_func_(tmp_angle, target_x, h, t);
    if (t > 10) {
      error_message_ = "flight time (" + std::to_string(t) + "s) too long";
      return false;
    }
    dh = target_h - h;
    aimed_h += dh;
    if (fabs(dh) < 0.001) {
      break;
    }
  }
  if (fabs(dh) > 0.01) {
    error_message_ = "height error (" + std::to_string(dh) + "m) too large, not converged";
    return false;
  }
  angle = tmp_angle;
  return true;
}

bool IterativeProjectileTool::solve_high(
  double target_x, double target_h, double speed, double gravity, double & angle)
{
  if (target_x <= 1e-6 || speed <= 0.0 || gravity <= 0.0) {
    error_message_ = "invalid range/speed/gravity";
    return false;
  }
  const double v2 = speed * speed;
  const double disc = v2 * v2 - gravity * (gravity * target_x * target_x + 2.0 * target_h * v2);
  if (disc < 0.0) {
    error_message_ = "no real ballistic root";
    return false;
  }
  const double high = std::atan((v2 + std::sqrt(disc)) / (gravity * target_x));
  if (high <= 45.0 * M_PI / 180.0 + 1e-6) {
    error_message_ = "high root is not a lob branch";
    return false;
  }

  // 下面循环是 ITL IterativeProjectileTool::solve 原文，只把 aimed_h 初值换成高抛根。
  double aimed_h = target_x * std::tan(high);
  double dh = 0;
  double tmp_angle = 0;
  double h = 0;
  double t = 0;

  for (int i = 0; i < max_iter_; i++) {
    tmp_angle = atan2(aimed_h, target_x);
    if (tmp_angle > 80 * M_PI / 180 || tmp_angle < -80 * M_PI / 180) {
      error_message_ = "iterative angle out of range (-80, 80) deg";
      return false;
    }
    forward_motion_func_(tmp_angle, target_x, h, t);
    if (t > 10) {
      error_message_ = "flight time (" + std::to_string(t) + "s) too long";
      return false;
    }
    dh = target_h - h;
    aimed_h += dh;
    if (fabs(dh) < 0.001) {
      break;
    }
  }
  if (fabs(dh) > 0.01) {
    error_message_ = "height error (" + std::to_string(dh) + "m) too large, not converged";
    return false;
  }
  angle = tmp_angle;
  return true;
}

}  // namespace projectile_motion
