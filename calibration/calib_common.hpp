#pragma once
// 英雄 handeye_calibration 定制点迁入同济无 ROS 标定：
// - ROS camera_info.yaml / 扁平 calibration.yaml 双读
// - 圆点 blob + 灰度 findCirclesGrid
// - 云台 yaw/pitch 轴不共点偏移

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace calib {

inline cv::Ptr<cv::SimpleBlobDetector> create_blob_detector()
{
  cv::SimpleBlobDetector::Params params;
  params.filterByArea = true;
  params.minArea = 20;
  params.maxArea = 5000;
  params.filterByCircularity = true;
  params.minCircularity = 0.6f;
  params.filterByInertia = true;
  params.minInertiaRatio = 0.3f;
  params.filterByConvexity = true;
  params.minConvexity = 0.5f;
  params.filterByColor = true;
  params.blobColor = 0;  // 白底黑点
  return cv::SimpleBlobDetector::create(params);
}

inline bool find_circle_grid(
  const cv::Mat & img, const cv::Size & pattern_size, std::vector<cv::Point2f> & centers_2d)
{
  cv::Mat gray;
  if (img.channels() == 1) {
    gray = img;
  } else {
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  }
  return cv::findCirclesGrid(
    gray, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID, create_blob_detector());
}

// 竖直圆点板：原点在板心，Y 右、Z 上（英雄手眼约定）
inline std::vector<cv::Point3f> centers_3d_vertical(
  const cv::Size & pattern_size, float center_distance)
{
  std::vector<cv::Point3f> points;
  for (int i = 0; i < pattern_size.height; i++) {
    for (int j = 0; j < pattern_size.width; j++) {
      float x = 0.0f;
      float y = (j - 0.5f * (pattern_size.width - 1)) * center_distance;
      float z = -(i - 0.5f * (pattern_size.height - 1)) * center_distance;
      points.push_back({x, y, z});
    }
  }
  return points;
}

// 平面板：角点原点在左上（同济 calibrateHandEye / 内参）
inline std::vector<cv::Point3f> centers_3d_planar(
  const cv::Size & pattern_size, float center_distance)
{
  std::vector<cv::Point3f> points;
  for (int i = 0; i < pattern_size.height; i++) {
    for (int j = 0; j < pattern_size.width; j++) {
      points.push_back({j * center_distance, i * center_distance, 0});
    }
  }
  return points;
}

inline Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  q_file >> w >> x >> y >> z;
  return {w, x, y, z};
}

inline Eigen::Vector3d read_vec3_mm(const YAML::Node & yaml, const char * key)
{
  if (!yaml[key]) return Eigen::Vector3d::Zero();
  auto v = yaml[key].as<std::vector<double>>();
  if (v.size() != 3) return Eigen::Vector3d::Zero();
  return {v[0], v[1], v[2]};
}

// 支持 ROS camera_info（嵌套 data）或扁平 camera_matrix / distort_coeffs
inline bool load_intrinsics(
  const std::string & path, cv::Mat & camera_matrix, cv::Mat & distort_coeffs, std::string * err = nullptr)
{
  try {
    YAML::Node yaml = YAML::LoadFile(path);
    std::vector<double> K, D;

    if (yaml["camera_matrix"] && yaml["camera_matrix"].IsMap() && yaml["camera_matrix"]["data"]) {
      K = yaml["camera_matrix"]["data"].as<std::vector<double>>();
      if (yaml["distortion_coefficients"] && yaml["distortion_coefficients"]["data"]) {
        D = yaml["distortion_coefficients"]["data"].as<std::vector<double>>();
      } else if (yaml["distort_coeffs"]) {
        D = yaml["distort_coeffs"].as<std::vector<double>>();
      }
    } else if (yaml["camera_matrix"] && yaml["camera_matrix"].IsSequence()) {
      K = yaml["camera_matrix"].as<std::vector<double>>();
      if (yaml["distort_coeffs"]) {
        D = yaml["distort_coeffs"].as<std::vector<double>>();
      } else if (yaml["distortion_coefficients"]) {
        D = yaml["distortion_coefficients"].as<std::vector<double>>();
      }
    } else {
      if (err) *err = "missing camera_matrix";
      return false;
    }

    if (K.size() != 9) {
      if (err) *err = fmt::format("camera_matrix size {} != 9", K.size());
      return false;
    }
    camera_matrix = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; i++) camera_matrix.at<double>(i / 3, i % 3) = K[i];

    distort_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    for (size_t i = 0; i < D.size() && i < 5; i++) {
      distort_coeffs.at<double>(0, static_cast<int>(i)) = D[i];
    }
    return true;
  } catch (const std::exception & e) {
    if (err) *err = e.what();
    return false;
  }
}

inline bool load_intrinsics_prefer(
  const std::string & camera_info_path, const std::string & config_path, cv::Mat & K, cv::Mat & D,
  std::string * err = nullptr)
{
  if (!camera_info_path.empty()) {
    return load_intrinsics(camera_info_path, K, D, err);
  }
  return load_intrinsics(config_path, K, D, err);
}

// yaw/pitch 轴不共点：云台原点在世界系下的平移（mm）
inline Eigen::Vector3d gimbal_origin_in_world_mm(
  const Eigen::Vector3d & ypr_rad, const Eigen::Vector3d & p_yaw_to_pitch_mm,
  const Eigen::Vector3d & p_pitch_to_barrel_mm)
{
  Eigen::Matrix3d R_yaw(Eigen::AngleAxisd(ypr_rad[0], Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d R_pitch(Eigen::AngleAxisd(ypr_rad[1], Eigen::Vector3d::UnitY()));
  return R_yaw * p_yaw_to_pitch_mm + R_yaw * R_pitch * p_pitch_to_barrel_mm;
}

inline double mean_reproj_error(
  const std::vector<cv::Point3f> & obj, const std::vector<cv::Point2f> & img, const cv::Mat & rvec,
  const cv::Mat & tvec, const cv::Mat & K, const cv::Mat & D)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(obj, rvec, tvec, K, D, projected);
  double total = 0;
  for (size_t i = 0; i < projected.size(); i++) total += cv::norm(projected[i] - img[i]);
  return projected.empty() ? 1e9 : total / projected.size();
}

inline void write_text(const std::string & path, const std::string & text)
{
  if (path.empty()) return;
  std::ofstream out(path);
  out << text;
  fmt::print("结果已保存至: {}\n", path);
}

inline std::string emit_camera_info_ros(
  int width, int height, const cv::Mat & K, const cv::Mat & D, double error)
{
  YAML::Emitter out;
  out << YAML::Comment(fmt::format("重投影误差: {:.4f}px", error));
  out << YAML::BeginMap;
  out << YAML::Key << "image_width" << YAML::Value << width;
  out << YAML::Key << "image_height" << YAML::Value << height;
  out << YAML::Key << "camera_name" << YAML::Value << "narrow_stereo";
  out << YAML::Key << "camera_matrix" << YAML::BeginMap;
  out << YAML::Key << "rows" << YAML::Value << 3;
  out << YAML::Key << "cols" << YAML::Value << 3;
  out << YAML::Key << "data" << YAML::Value << YAML::Flow
      << std::vector<double>(K.begin<double>(), K.end<double>());
  out << YAML::EndMap;
  out << YAML::Key << "distortion_model" << YAML::Value << "plumb_bob";
  out << YAML::Key << "distortion_coefficients" << YAML::BeginMap;
  out << YAML::Key << "rows" << YAML::Value << 1;
  out << YAML::Key << "cols" << YAML::Value << 5;
  std::vector<double> d(5, 0.0);
  for (int i = 0; i < std::min(5, D.cols * D.rows); i++) d[i] = D.at<double>(i);
  out << YAML::Key << "data" << YAML::Value << YAML::Flow << d;
  out << YAML::EndMap;
  out << YAML::EndMap;
  return out.c_str();
}

inline std::string emit_camera_flat(const cv::Mat & K, const cv::Mat & D, double error)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Comment(fmt::format("重投影误差: {:.4f}px", error));
  out << YAML::Key << "camera_matrix" << YAML::Value << YAML::Flow
      << std::vector<double>(K.begin<double>(), K.end<double>());
  out << YAML::Key << "distort_coeffs" << YAML::Value << YAML::Flow
      << std::vector<double>(D.begin<double>(), D.end<double>());
  out << YAML::Newline << YAML::EndMap;
  return out.c_str();
}

}  // namespace calib
