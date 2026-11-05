#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace auto_aim
{
constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

void Solver::solve(Armor & armor) const
{
  const auto & object_points = (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  // 平衡不做yaw优化，因为pitch假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four || armor.name == ArmorName::five);
  tools::Plotter plotter;
  nlohmann::json data;
  data["pnp_x"] = armor.xyz_in_world[0];
  data["pnp_y"] = armor.xyz_in_world[1];
  data["pnp_z"] = armor.xyz_in_world[2];
  data["pnp_d"] = std::sqrt(armor.xyz_in_world[0] * armor.xyz_in_world[0] + armor.xyz_in_world[1] * armor.xyz_in_world[1]);
  plotter.plot(data);

  if (is_balance) return;

  // optimize_yaw(armor);
}

std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

void Solver::optimize_yaw(Armor & armor, double yaw) const
{
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  auto range = 120;  // degree
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - range / 2 * CV_PI / 180.0);

  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < range; i++) {
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    // auto error = armor_reprojection_error(armor, yaw, std::abs(std::abs(yaw) - std::abs(std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]))));
    // auto error = armor_reprojection_error(armor, yaw);
    // tools::logger()->debug("armor yaw = {:.2f}, gimbal yaw = {:.2f}", tools::limit_rad(armor.ypr_in_gimbal[0]) * 57.3,tools::limit_rad(yaw)*57.3);
    // tools::logger()->debug("inclined = {}", std::abs(tools::limit_rad(armor.ypr_in_gimbal[0])) * 57.3);
    auto error = armor_reprojection_error(armor, yaw, std::abs((i - range / 2) * CV_PI / 180.0));

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
}

double Solver::armor_reprojection_error(const Armor & armor, double yaw) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);

  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}

double Solver::armor_reprojection_error(const Armor & armor, double yaw, double inclined) const
{
  auto cv_refs = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto cv_pts = armor.points;
  std::size_t size = cv_refs.size();
  if (size != cv_pts.size() || size < 2) {
    throw std::invalid_argument("Input vectors must have the same size and at least 2 points.");
  }

  double cost = 0.0;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;  // 形成闭合形状

    // 计算相邻点形成的线段向量
    cv::Point2f ref_d = cv_refs[p] - cv_refs[i];  // 参考形状的边
    cv::Point2f pt_d = cv_pts[p] - cv_pts[i];     // 目标形状的边

    // 计算欧几里得距离（点误差）
    double ref_norm = std::hypot(ref_d.x, ref_d.y);
    double pt_norm = std::hypot(pt_d.x, pt_d.y);
    double pixel_dis =
      (0.5 * (cv::norm(cv_refs[i] - cv_pts[i]) + cv::norm(cv_refs[p] - cv_pts[p])) + std::fabs(ref_norm - pt_norm)) / ref_norm;

    // 计算角度误差
    double dot_product = ref_d.x * pt_d.x + ref_d.y * pt_d.y;
    double cos_angle = dot_product / (ref_norm * pt_norm + 1e-6);  // 避免除零
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));          // 限制在 [-1,1] 之间
    double angular_dis = ref_norm * std::acos(cos_angle) / ref_norm;

    // tools::logger()->debug("when inclined = {:.2f}, pix = {:.2f}, ang = {:.2f}",
    //                         std::abs(tools::limit_rad(armor.ypr_in_gimbal[0])) * 57.3,
    //                         std::pow(pixel_dis * std::sin(inclined), 2),
    //                         std::pow(angular_dis * std::cos(inclined), 2) * 1
    //                       );
    // 计算加权误差
    double cost_i =
      std::pow(pixel_dis * std::sin(inclined), 2) + std::pow(angular_dis * std::cos(inclined), 2) * 2;  // 最后一个参数2可调

    // 取平方根并累加
    cost += std::sqrt(cost_i);
  }
  return cost;
}

}  // namespace auto_aim