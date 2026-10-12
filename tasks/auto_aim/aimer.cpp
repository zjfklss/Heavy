#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
namespace
{
// EKF: x vx y vy z vz a w r l h（见 target.cpp）。同济旧码陀螺分支误用 [8]=r，此处固定用 w。
constexpr int kIdxW = 7;
constexpr int kIdxR = 8;  // 半径；选板勿当转速
}  // namespace

Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }

  // 钉头：抄远程 jlauto-aim/hero opti_solver processAssessment2 / processOutpost
  if (yaml["nail_mode"].IsDefined()) nail_mode_ = yaml["nail_mode"].as<bool>();
  if (yaml["nail_spin_on"].IsDefined()) nail_spin_on_ = yaml["nail_spin_on"].as<double>();
  if (yaml["nail_spin_off"].IsDefined()) nail_spin_off_ = yaml["nail_spin_off"].as<double>();
  if (yaml["nail_fire_window_deg"].IsDefined())
    nail_fire_window_ = yaml["nail_fire_window_deg"].as<double>() / 57.3;
  if (nail_spin_off_ > nail_spin_on_) nail_spin_off_ = nail_spin_on_ * 0.7;
  if (nail_mode_) {
    tools::logger()->info(
      "[Aimer] nail_mode on: spin_on={:.2f} spin_off={:.2f} fire_window_deg={:.1f}", nail_spin_on_,
      nail_spin_off_, nail_fire_window_ * 57.3);
  }
}

void Aimer::update_nail_latch(double abs_w)
{
  if (!nail_latched_ && abs_w >= nail_spin_on_) nail_latched_ = true;
  if (nail_latched_ && abs_w <= nail_spin_off_) nail_latched_ = false;
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();

  // |w|：同济 delay 曾用带符号 w，反向高速误走低速延时；planner 已 abs，此处对齐
  const double abs_w = std::abs(target.ekf_x()[kIdxW]);
  double delay_time = abs_w > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  if (bullet_speed < 14) bullet_speed = 23;

  auto future = timestamp;
  if (to_now) {
    double dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  } else {
    auto dt = 0.005 + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  update_nail_latch(std::abs(target.ekf_x()[kIdxW]));
  if (nail_mode_ && nail_latched_) return aim_nail(target, future, bullet_speed);
  return aim_track(target, future, bullet_speed);
}

io::Command Aimer::aim_track(
  Target target, std::chrono::steady_clock::time_point future, double bullet_speed)
{
  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) return {false, false, 0, 0};

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  std::vector<Target> iteration_target(10, target);

  for (int iter = 0; iter < 10; ++iter) {
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].predict(predict_time);

    auto aim_point = choose_aim_point(iteration_target[iter]);
    debug_aim_point = aim_point;
    if (!aim_point.valid) return {false, false, 0, 0};

    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) break;
    prev_fly_time = current_traj.fly_time;
  }

  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = tools::limit_rad(std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_);
  double pitch = -(current_traj.pitch + pitch_offset_);
  return {true, false, yaw, pitch};
}

io::Command Aimer::aim_nail(
  Target target, std::chrono::steady_clock::time_point future, double bullet_speed)
{
  // 对照 hero processAssessment2：
  // 1) 迭代飞行时间；2) yaw 钉预测车中心；3) 选最正对装甲解 pitch；4) 窗口内开火
  auto x0 = target.ekf_x();
  double xc = x0[0], yc = x0[2], zc = x0[4];
  double d0 = std::hypot(xc, yc);
  tools::Trajectory traj0(bullet_speed, d0, zc);
  if (traj0.unsolvable) {
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  double prev_fly = traj0.fly_time;
  tools::Trajectory current_traj = traj0;
  Target work = target;
  double min_diff = 1e9;
  Eigen::Vector4d best_xyza = Eigen::Vector4d::Zero();
  bool best_ok = false;

  for (int iter = 0; iter < 10; ++iter) {
    auto t_hit = future + std::chrono::microseconds(static_cast<int>(prev_fly * 1e6));
    work = target;
    work.predict(t_hit);
    auto x = work.ekf_x();
    xc = x[0];
    yc = x[2];
    zc = x[4];
    const double center_view_yaw = std::atan2(yc, xc);
    const double r = x[kIdxR];

    auto armor_list = work.armor_xyza_list();
    min_diff = 1e9;
    int best_i = 0;
    for (size_t i = 0; i < armor_list.size(); ++i) {
      // 装甲朝向相对视线：与 hero shortest_angular_distance(plate_yaw, center_view) 同义
      double diff = std::abs(tools::limit_rad(armor_list[i][3] - center_view_yaw));
      if (diff < min_diff) {
        min_diff = diff;
        best_i = static_cast<int>(i);
      }
    }
    best_xyza = armor_list[best_i];
    best_ok = true;

    // pitch 靶用命中时刻最正对装甲（Assessment2）；yaw 另取车中心
    Eigen::Vector3d xyz = best_xyza.head(3);
    (void)r;
    double d = std::hypot(xyz.x(), xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());
    if (current_traj.unsolvable) {
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }
    if (std::abs(current_traj.fly_time - prev_fly) < 0.001) break;
    prev_fly = current_traj.fly_time;
  }

  if (!best_ok) {
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  debug_aim_point = {true, best_xyza};
  const double yaw_cmd = tools::limit_rad(std::atan2(yc, xc) + yaw_offset_);
  const double pitch_cmd = -(current_traj.pitch + pitch_offset_);
  const bool shoot = min_diff < nail_fire_window_;

  tools::logger()->debug(
    "[Aimer] nail: min_diff={:.1f}deg window={:.1f}deg shoot={} yaw={:.2f}", min_diff * 57.3,
    nail_fire_window_ * 57.3, shoot, yaw_cmd);
  return {true, shoot, yaw_cmd, pitch_cmd};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();
  if (!target.jumped) return {true, armor_xyza_list[0]};

  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  std::vector<double> delta_angle_list;
  for (int i = 0; i < static_cast<int>(armor_num); i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 低速/非陀螺选板：|w|<=2。同济原版写 ekf[8]（半径）→ 几乎永远进此分支，见上游 #15
  const double abs_w = std::abs(ekf_x[kIdxW]);
  if (abs_w <= 2 && target.name != ArmorName::outpost) {
    std::vector<int> id_list;
    for (int i = 0; i < static_cast<int>(armor_num); i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;
      return {true, armor_xyza_list[lock_id_]};
    }

    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  for (int i = 0; i < static_cast<int>(armor_num); i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[kIdxW] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
    if (ekf_x[kIdxW] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
  }

  return {false, armor_xyza_list[0]};
}

}  // namespace auto_aim
