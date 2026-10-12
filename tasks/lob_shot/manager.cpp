#include "manager.hpp"

#include <cmath>

#include "projectile_motion/gaf_projectile_solver.hpp"
#include "windmill.hpp"

namespace lob_shot
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

Manager::Manager(Config cfg) : cfg_(cfg) {}

void Manager::request_trigger() { trigger_pending_ = true; }

void Manager::request_cancel()
{
  clear_aim_request();
  if (state_ != State::IDLE && state_ != State::INHIBITED) {
    set_state(State::IDLE, "IDLE");
  }
}

void Manager::set_fire(bool on) { fire_ = on; }

void Manager::set_gate(bool unlocked, bool docked, bool assembling)
{
  cfg_.module_unlocked = unlocked;
  cfg_.module_docked = docked;
  cfg_.module_assembling = assembling;
}

void Manager::set_feedback(double yaw, double pitch)
{
  fb_yaw_ = yaw;
  fb_pitch_ = pitch;
}

void Manager::set_bullet_speed(double v)
{
  if (v > 1.0) cfg_.bullet_speed = v;
}

bool Manager::gate_ok() const
{
  return cfg_.module_unlocked && cfg_.module_docked && !cfg_.module_assembling;
}

void Manager::set_state(State s, const std::string & name)
{
  state_ = s;
  status_ = name;
  t_in_state_ = 0;
}

void Manager::clear_aim_request()
{
  trigger_pending_ = false;
  fire_ = false;
}

void Manager::solve_yaw()
{
  const double c = std::cos(cfg_.base_yaw);
  const double s = std::sin(cfg_.base_yaw);
  // 枪口在 map：底盘位姿 ⊕ 车体偏移
  const double mx = cfg_.base_x + c * cfg_.muzzle_dx - s * cfg_.muzzle_dy;
  const double my = cfg_.base_y + s * cfg_.muzzle_dx + c * cfg_.muzzle_dy;
  const double dx = cfg_.target_x - mx;
  const double dy = cfg_.target_y - my;
  target_yaw_map_ = std::atan2(dy, dx);
  // 相对底盘（对齐 Heavy solveYaw）
  target_yaw_ = wrap_pi((target_yaw_map_ - cfg_.base_yaw) * cfg_.yaw_sign);
  last_yaw_deg_ = target_yaw_ * 180.0 / kPi;
}

bool Manager::hits_windmill(double mx, double my, double mz) const
{
  for (const auto & p : traj_xz_) {
    const double x = mx + p.first * std::cos(target_yaw_map_);
    const double y = my + p.first * std::sin(target_yaw_map_);
    const double z = mz + p.second;
    if (point_in_cylinder(
          x, y, z, cfg_.windmill_x, cfg_.windmill_y, cfg_.windmill_radius, cfg_.windmill_z_min,
          cfg_.windmill_z_max)) {
      return true;
    }
  }
  return false;
}

void Manager::build_traj(double pitch)
{
  projectile_motion::GafProjectileSolver solver(cfg_.bullet_speed, cfg_.friction);
  traj_xz_ = solver.computeTrajectory(pitch, last_dist_, 40);
}

bool Manager::solve_pitch_low_only()
{
  const double c = std::cos(cfg_.base_yaw);
  const double s = std::sin(cfg_.base_yaw);
  const double mx = cfg_.base_x + c * cfg_.muzzle_dx - s * cfg_.muzzle_dy;
  const double my = cfg_.base_y + s * cfg_.muzzle_dx + c * cfg_.muzzle_dy;
  const double mz = cfg_.base_z + cfg_.muzzle_dz;
  const double dx = cfg_.target_x - mx;
  const double dy = cfg_.target_y - my;
  last_dist_ = std::hypot(dx, dy);
  const double dh = cfg_.target_z - mz;

  projectile_motion::GafProjectileSolver solver(cfg_.bullet_speed, cfg_.friction);
  double high = 0;
  double low = 0;
  solver.solve_high(last_dist_, dh, high);
  if (!solver.solve(last_dist_, dh, low)) {
    set_state(State::IDLE, "IDLE (ballistic fail)");
    clear_aim_request();
    return false;
  }
  last_high_deg_ = high * 180.0 / kPi;
  const double pitch_max = cfg_.pitch_max_deg * kPi / 180.0;
  if (low > pitch_max + 1e-6) {
    last_pitch_deg_ = low * 180.0 / kPi;
    set_state(State::IDLE, "IDLE (low exceeds pitch_max)");
    clear_aim_request();
    return false;
  }
  target_pitch_ = low * cfg_.pitch_sign;
  last_pitch_deg_ = low * 180.0 / kPi;
  build_traj(low);
  traj_blocked_ = hits_windmill(mx, my, mz);
  if (traj_blocked_) {
    set_state(State::IDLE, "IDLE (windmill block)");
    clear_aim_request();
    return false;
  }
  return true;
}

Command Manager::tick(double dt_s)
{
  Command cmd;
  t_in_state_ += dt_s;

  // P0：门闩关闭必须清 trigger/fire，禁止恢复后自动开火
  if (!gate_ok()) {
    if (state_ != State::INHIBITED) {
      clear_aim_request();
      set_state(State::INHIBITED, "INHIBITED");
    }
    return cmd;
  }
  if (state_ == State::INHIBITED) {
    set_state(State::IDLE, "IDLE");
  }

  switch (state_) {
    case State::IDLE:
      if (trigger_pending_) {
        trigger_pending_ = false;  // 边沿消费
        set_state(State::WAITING_POSE, "WAITING_POSE");
      }
      break;
    case State::WAITING_POSE:
      // 先验位姿已在 cfg；预留超时接口
      if (t_in_state_ > cfg_.pose_timeout_s) {
        set_state(State::IDLE, "IDLE (pose timeout)");
        clear_aim_request();
        break;
      }
      set_state(State::YAW_AIMING, "YAW_AIMING");
      break;
    case State::YAW_AIMING:
      solve_yaw();
      set_state(State::YAW_CONVERGING, "YAW_CONVERGING");
      cmd.control = true;
      cmd.yaw = target_yaw_;
      cmd.pitch = fb_pitch_;  // 转 yaw 期间保持当前 pitch（对齐 Heavy）
      break;
    case State::YAW_CONVERGING: {
      cmd.control = true;
      cmd.yaw = target_yaw_;
      cmd.pitch = fb_pitch_;
      const double ey = wrap_pi(fb_yaw_ - target_yaw_);
      if (cfg_.open_loop || std::fabs(ey) < cfg_.yaw_tol) {
        set_state(State::PITCH_AIMING, "PITCH_AIMING");
      } else if (t_in_state_ > cfg_.aim_timeout_s) {
        set_state(State::IDLE, "IDLE (yaw timeout)");
        clear_aim_request();
        cmd = Command{};
      }
      break;
    }
    case State::PITCH_AIMING:
      if (solve_pitch_low_only()) {
        set_state(State::PITCH_CONVERGING, "PITCH_CONVERGING");
        cmd.control = true;
        cmd.yaw = target_yaw_;
        cmd.pitch = target_pitch_;
      }
      break;
    case State::PITCH_CONVERGING: {
      cmd.control = true;
      cmd.yaw = target_yaw_;
      cmd.pitch = target_pitch_;
      if (cfg_.open_loop || std::fabs(fb_pitch_ - target_pitch_) < cfg_.pitch_tol) {
        set_state(State::READY, "READY");
        cmd.shoot = fire_;
      } else if (t_in_state_ > cfg_.aim_timeout_s) {
        set_state(State::IDLE, "IDLE (pitch timeout)");
        clear_aim_request();
        cmd = Command{};
      }
      break;
    }
    case State::READY:
      // READY 再来 trigger → 重新解算（对齐 Heavy）
      if (trigger_pending_) {
        trigger_pending_ = false;
        set_state(State::WAITING_POSE, "WAITING_POSE");
        break;
      }
      cmd.control = true;
      cmd.shoot = fire_;
      cmd.yaw = target_yaw_;
      cmd.pitch = target_pitch_;
      break;
    case State::INHIBITED:
      break;
  }
  return cmd;
}

}  // namespace lob_shot
