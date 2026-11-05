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

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  uint8_t armor_state = 0;  //用位图管理
  bool jumped;
  int last_id;  // debug only

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
