#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <deque>
#include <array>
#include <optional>
#include <functional>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

/*
 * Target：对一个敌方整车(或前哨站/基地)的扩展卡尔曼滤波(EKF)跟踪。
 *
 * EKF 状态向量(11维)： x vx y vy z vz a w r l h
 *   x,y,z : 整车旋转中心(不是装甲板！)的世界系坐标；v 为速度
 *   a     : 其中一块装甲板的朝向角；w 为整车角速度(rad/s，正负表示转向)
 *   r     : 旋转半径；l,h : 另一对装甲板相对的半径差和高度差(仅 4 板车使用)
 * 每块装甲板的位置由中心 + 半径 + 角度推出(见 h_armor_xyz)。
 *
 * 状态机：LOST -> DETECTING(连续识别) -> TRACKING；TRACKING 丢帧 -> TEMP_LOST，
 * 丢帧超过 max_temp_lost_count 才回到 LOST。LOST 时滤波器会被整个重置。
 * 前哨站每转一圈会有板子短暂看不见，所以它的 temp_lost 上限单独设得更大(tracker.cpp)。
 *
 * 前哨站特例：w 在加锁时由 set_w() 一次性设定(符号取观测方向，大小固定 0.8π)，
 * 且过程噪声为 0，滤波器之后不会再改它；转向反了要靠 monitor_outpost_direction() 纠正。
 */
class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  uint8_t armor_state = 0;  //用位图管理
  bool jumped;
  int last_id;  // debug only
  int temp_lost_count() const { return temp_lost_count_; }

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num, Eigen::VectorXd P0_dig,
    double v1 = 100, double v2 = 400, double w = 0);
  Target(
    int armor_num, double r_, Eigen::VectorXd P0_dig, int min_detect_count, int max_temp_lost_count, double v1 = 100,
    double v2 = 400, double w = 0);

  void predict(std::chrono::steady_clock::time_point t);
  void update(const Armor & armor, std::chrono::steady_clock::time_point t);
  void update(const Armor * armor, std::chrono::steady_clock::time_point t);
  const std::array<std::optional<double>, 3> & layer_heights() const { return layer_heights_; }

  Eigen::VectorXd ekf_x() const;
  tools::ExtendedKalmanFilter ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;
  bool state();
  std::string state_str();
  void clear_target();

private:
  enum State { LOST, DETECTING, TEMP_LOST, TRACKING };
  double v1_;
  double v2_;
  int armor_num_;

  int min_detect_count_ = 0;
  int max_temp_lost_count_ = 0;
  int detect_count_ = 0;
  int temp_lost_count_ = 0;
  State state_ = LOST;
  Eigen::MatrixXd P0_;
  double w_;
  double r_;
  int need_set_w_;
  // 前哨站转向监测：w 在加锁时一次性设定且过程噪声为0，前哨站反转后不会自己改回来
  bool dir_has_last_ = false;
  double dir_last_yaw_ = 0;
  std::chrono::steady_clock::time_point dir_last_t_;
  double dir_acc_ = 0;     // 窗口内装甲板朝向净转角 rad
  double dir_time_ = 0;    // 窗口已累计时间 s
  int dir_bad_windows_ = 0;
  void monitor_outpost_direction(const Armor & armor, std::chrono::steady_clock::time_point t);
  std::array<std::optional<double>, 3> layer_heights_{};
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  int transition(const Armor * armor, std::chrono::steady_clock::time_point t);
  void set_target(const Armor * armor, std::chrono::steady_clock::time_point t);
  void set_w();

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
