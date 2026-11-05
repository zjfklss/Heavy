#include "target.hpp"

#include <iostream>
#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
// 把 w、v1、v2 传进来
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num, Eigen::VectorXd P0_dig, double v1,
  double v2, double w)
: name(armor.name), armor_type(armor.type), jumped(false), last_id(0), armor_num_(armor_num), t_(t), v1_(v1), v2_(v2)
{
  auto r = radius;

  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], w, r, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
  if (name == outpost) layer_heights_[0] = xyz[2];
}

Target::Target(
  int armor_num, double r, Eigen::VectorXd P0_dig, int min_detect_count, int max_temp_lost_count, double v1, double v2,
  double w)
: armor_num_(armor_num),
  min_detect_count_(min_detect_count),
  max_temp_lost_count_(max_temp_lost_count),
  r_(r),
  v1_(v1),
  v2_(v2),
  w_(w)
{
  state_ = LOST;
  P0_ = P0_dig.asDiagonal();
  x_add_ = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };
}

void Target::set_target(
  const Armor * armor, std::chrono::steady_clock::time_point t)  // 从LOST进入DETECTING后，设置本次识别到的目标的初值
{
  tools::logger()->debug("---------- new target set!:{} ----------", static_cast<int>(armor->name));
  name = armor->name;
  armor_type = armor->type;
  need_set_w_ = (name == outpost ? 1 : 0);
  jumped = false;
  last_id = 0;
  layer_heights_ = {};
  temp_lost_count_ = 0;
  t_ = t;
  const Eigen::VectorXd & xyz = armor->xyz_in_world;
  const Eigen::VectorXd & ypr = armor->ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r_ * std::cos(ypr[0]);
  auto center_y = xyz[1] + r_ * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], w_, r_, 0, 0}};

  ekf_ = tools::ExtendedKalmanFilter(x0, P0_, x_add_);
  // tools::logger()->info("set to: {}", ekf_x()[7]);
}

void Target::set_w()  // 前哨站独有，用前五帧的结果给一个w，防止远距离不认为它在旋转
{
  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  auto x = ekf_x();
  double w_temp;
  Eigen::MatrixXd P0_temp;

  if (std::abs(x[7]) > 0.1) {
    w_temp = ((x[7] > 0) ? 1 : -1) * 0.8 * M_PI;
    Eigen::VectorXd new_P0_dig_outpost{{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.4, 0.01, 0.0001, 0, 0}};
    P0_temp = new_P0_dig_outpost.asDiagonal();
  } else {
    w_temp = 0;
    Eigen::VectorXd new_P0_dig_outpost{{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.4, 0.01, 0.0001, 0, 0}};
    P0_temp = new_P0_dig_outpost.asDiagonal();
  }
  Eigen::VectorXd new_x0{{x[0], 0, x[2], 0, x[4], 0, x[6], w_temp, x[8], x[9], x[10]}};

  ekf_ = tools::ExtendedKalmanFilter(new_x0, P0_temp, x_add_);
  need_set_w_ = 0;
  tools::logger()->info("w_ set to: {} {}", w_temp, x[7]);
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  t_ = t;

  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  auto v1 = v1_;  // 加速度方差
  auto v2 = v2_;  // 角加速度方差
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt / 1;
  // clang-format off
  // x vx y vy z vz a w r1 r2 h
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  ekf_.predict(F, Q, f);
}
void Target::clear_target()
{
  state_ = LOST;
  detect_count_ = 0;
  temp_lost_count_ = 0;
  need_set_w_ = 0;
  layer_heights_ = {};
}

int Target::transition(const Armor * armor, std::chrono::steady_clock::time_point t)
{
  int flag;
  switch (state_) {
    case LOST:
      if (armor != NULL) {
        state_ = DETECTING;
        detect_count_ = 1;
        set_target(armor, t);
      }
      break;
    case DETECTING:
      if (armor != NULL) {
        detect_count_++;
      } else {
        detect_count_ = 0;
        state_ = LOST;
        need_set_w_ = 0;
      }

      if (detect_count_ > min_detect_count_) {
        state_ = TRACKING;
        detect_count_ = 0;
        if (need_set_w_) {
          set_w();
        }
      }
      break;
    case TEMP_LOST:
      if (armor != NULL) {
        state_ = TRACKING;
        temp_lost_count_ = 0;
        if (need_set_w_) {
          set_w();
        }
      } else {
        temp_lost_count_++;
      }
      if (temp_lost_count_ > max_temp_lost_count_) {
        state_ = LOST;
        need_set_w_ = 0;
        detect_count_ = 0;
        temp_lost_count_ = 0;
      }
      break;
    case TRACKING:
      if (armor == NULL) {
        temp_lost_count_ = 1;
        state_ = TEMP_LOST;
      }
      break;
  }
  return flag = (armor == NULL) ? 0 : 1;
}

void Target::update(const Armor & armor, std::chrono::steady_clock::time_point t)
{
  update(&armor, t);
}

void Target::update(const Armor * observation, std::chrono::steady_clock::time_point t)
{
  // 状态更新
  auto past = state_str();
  if (transition(observation, t) == 0) return;
  const auto & armor = *observation;

  // 装甲板匹配
  int id;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  for (int i = 0; i < armor_num_; i++) {
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza_list[i].head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza_list[i][3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = i;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped = true;
  last_id = id;
  if (name == outpost && id >= 0 && id < 3 && std::isfinite(armor.xyz_in_world[2]))
    layer_heights_[id] = armor.xyz_in_world[2];
  // tools::logger()->debug("id = {}", id);

  update_ypda(armor, id);

  // 发散检测
  if (state_ != LOST && diverged()) {
    tools::logger()->debug("[Target] Target diverged!");
    state_ = LOST;
  }

  // 收敛效果检测：
  if (std::accumulate(ekf_.recent_nis_failures.begin(), ekf_.recent_nis_failures.end(), 0) >= (0.4 * ekf_.window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = LOST;  // todo 可能要换成更好的重启滤波器方法
  }
}

void Target::update_ypda(const Armor & armor, int id)
{
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  // {4e-3, 4e-3, 1, 2e-1}
  Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 2e-1}};  // 观测噪声 yaw pitch distance angle, 前三个是球坐标系坐标，最后一个是朝向
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

  ekf_.update(z, H, R, h, z_subtract);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

tools::ExtendedKalmanFilter Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    if (name == outpost && armor_num_ == 3 && layer_heights_[i]) xyz[2] = *layer_heights_[i];
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }

  return _armor_xyza_list;
}

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::state() { return (state_ == LOST) ? 0 : 1; }

std::string Target::state_str()
{
  switch (state_) {
    case (LOST):
      return "LOST";
      break;
    case (DETECTING):
      return "DETECTING";
      break;
    case (TEMP_LOST):
      return "TEMP_LOST";
      break;
    case (TRACKING):
      return "TRACKING";
      break;
  }
  return "LOST";
}

}  // namespace auto_aim
