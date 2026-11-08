// Copyright 2022 Chen Jun

#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <limits>
#include <vector>
#include <cmath>

namespace rm_auto_aim
{
/**
 * @file pnp_solver.cpp
 * @brief PnP 求解器的实现：将装甲的图像顶点映射为相机坐标系下的位姿。
 *
 * 构造函数将相机内参和畸变系数转换为 OpenCV 的 cv::Mat 格式，并初始化
 * 小/大装甲的 3D 模型点（以米为单位）。
 */
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  /// 定义小/大装甲的3D模型点，单位为米
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;

  // constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0 * cos(15/180*M_PI);
  // constexpr double small_half_x = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0 * sin(15/180*M_PI);

  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;
  // constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0 * cos(15/180*M_PI);
  // constexpr double large_half_x = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0 * sin(15/180*M_PI);

  /// 以装甲板中心为原点，从装甲板左下角开始，按顺时针顺序定义4个3D模型点
  /// 模型坐标系：x轴指向前方，y轴指向左侧，z轴指向上方
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  // outpost_armor_points_.emplace_back(cv::Point3f(small_half_x, small_half_y, -small_half_z));
  // outpost_armor_points_.emplace_back(cv::Point3f(-small_half_x, small_half_y, small_half_z));
  // outpost_armor_points_.emplace_back(cv::Point3f(-small_half_x, -small_half_y, small_half_z));
  // outpost_armor_points_.emplace_back(cv::Point3f(small_half_x, -small_half_y, -small_half_z));

  /** 
  outpost_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  outpost_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  outpost_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  outpost_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));
  * rensy注释**/
  
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));

  // large_armor_points_.emplace_back(cv::Point3f(-large_half_x, large_half_y, -large_half_z));
  // large_armor_points_.emplace_back(cv::Point3f(large_half_x, large_half_y, large_half_z));
  // large_armor_points_.emplace_back(cv::Point3f(large_half_x, -large_half_y, large_half_z));
  // large_armor_points_.emplace_back(cv::Point3f(-large_half_x, -large_half_y, -large_half_z));
}

bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_armor_points;

  // y轴图像偏移量,手动在图像坐标系中进行补偿
  // float offset_y = 68;

  // cv::Point2f left_bottom, left_top, right_bottom, right_top;
  // left_bottom = armor.left_light.bottom;
  // left_top = armor.left_light.top;
  // right_top = armor.right_light.top;
  // right_bottom = armor.right_light.bottom;

  // /******************进行补偿*****************/
  // left_bottom.y = left_bottom.y + offset_y;
  // left_top.y = left_top.y + offset_y;
  // right_top.y = right_top.y + offset_y;
  // right_bottom.y = right_bottom.y + offset_y;
  // /******************进行补偿*****************/

  //   // Fill in image point
  // image_armor_points.emplace_back(left_bottom);
  // image_armor_points.emplace_back(left_top);
  // image_armor_points.emplace_back(right_top);
  // image_armor_points.emplace_back(right_bottom);

  /// 填充图像点，顺序与3D模型点对应  
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  /// 基于装甲板类型选择对应的3D模型点，调用OpenCV的solvePnP函数进行求解
  // rvec = last_rvec;
  // tvec = last_tvec;
  // std::cout << camera_matrix_.at<double>(0,1) << std::endl;
  // std::cout << (armor.type == ArmorType::SMALL) << std::endl;
  auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_: large_armor_points_;
  return cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_ITERATIVE);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

float PnPSolver::calcReprojectionError(
  const Armor & armor, const cv::Mat & rvec, const cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_armor_points;
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  auto object_points = armor.type == ArmorType::SMALL
                       ? small_armor_points_ : large_armor_points_;

  std::vector<cv::Point2f> projected;
  cv::projectPoints(
      object_points, rvec, tvec,
      camera_matrix_, dist_coeffs_, projected);

  float total = 0.0f;
  for (size_t i = 0; i < image_armor_points.size(); i++) {
      total += static_cast<float>(cv::norm(image_armor_points[i] - projected[i]));
  }
  return total / static_cast<float>(image_armor_points.size());
}

bool PnPSolver::solvePnPPCABA(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_armor_points;
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  auto object_points = armor.type == ArmorType::SMALL
                       ? small_armor_points_ : large_armor_points_;

  // ===== Step 1: IPPE 求出全部候选解 =====
  // 平面 4 点 PnP 存在两个重投影误差相近的解（yaw 镜像翻转），
  // SOLVEPNP_IPPE 专为共面点设计，会一次性返回两个解及各自重投影误差。
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> reproj_errors;
  int n_sol = 0;
  try {
    n_sol = cv::solvePnPGeneric(
      object_points, image_armor_points, camera_matrix_, dist_coeffs_,
      rvecs, tvecs, false, cv::SOLVEPNP_IPPE,
      cv::noArray(), cv::noArray(), reproj_errors);
  } catch (const cv::Exception &) {
    n_sol = 0;
  }

  // 退化兜底：IPPE 不可用或无解时，回退到迭代法单解，保证不回归。
  if (n_sol <= 0 || rvecs.empty()) {
    bool ok = cv::solvePnP(
      object_points, image_armor_points, camera_matrix_, dist_coeffs_,
      rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (ok) {
      last_solutions_[armor.number] = {rvec.clone(), tvec.clone()};
    }
    return ok;
  }

  // ===== Step 2: 消歧，选定一个候选解 =====
  // 找重投影误差最小的解作为兜底基准
  int best_err_idx = 0;
  for (int i = 1; i < n_sol; ++i) {
    if (reproj_errors[i] < reproj_errors[best_err_idx]) {
      best_err_idx = i;
    }
  }

  int chosen = best_err_idx;

  auto hist_it = last_solutions_.find(armor.number);
  if (n_sol > 1 && hist_it != last_solutions_.end()) {
    // 时序一致性：选与上一帧旋转测地距离最小的解
    cv::Mat R_prev;
    cv::Rodrigues(hist_it->second.first, R_prev);

    int best_time_idx = 0;
    double best_geo = std::numeric_limits<double>::max();
    for (int i = 0; i < n_sol; ++i) {
      cv::Mat R_i;
      cv::Rodrigues(rvecs[i], R_i);
      double geo = rotationGeodesicDistance(R_i, R_prev);
      if (geo < best_geo) {
        best_geo = geo;
        best_time_idx = i;
      }
    }

    // 小陀螺/换面等真实跳变：所有候选解都离上一帧很远，
    // 此时时序约束无意义，直接采用重投影误差最小解。
    if (best_geo > MAX_ROT_JUMP) {
      chosen = best_err_idx;
    } else if (
      reproj_errors[best_time_idx] >
      reproj_errors[best_err_idx] * REPROJ_FALLBACK_RATIO)
    {
      // 时序最近解的重投影误差显著偏大，疑似跟错分支，回退到误差最小解
      chosen = best_err_idx;
    } else {
      chosen = best_time_idx;
    }
  }

  rvecs[chosen].copyTo(rvec);
  tvecs[chosen].copyTo(tvec);

  // ===== Step 3: ITERATIVE 抛光 =====
  // 以选定解为初值做一次迭代优化，进一步降低重投影误差。
  // useExtrinsicGuess=true 保证只在该分支邻域内收敛，不会跳到另一解。
  cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_,
    rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);

  // ===== Step 4: 按编号缓存历史 =====
  last_solutions_[armor.number] = {rvec.clone(), tvec.clone()};

  return true;
}

double PnPSolver::rotationGeodesicDistance(const cv::Mat & R1, const cv::Mat & R2)
{
  cv::Mat R = R1.t() * R2;
  double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
  double cos_angle = (trace - 1.0) / 2.0;
  cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
  return std::acos(cos_angle);
}

}  // namespace rm_auto_aim