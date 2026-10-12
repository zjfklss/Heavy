#pragma once
// 小圆点板主路径（英雄 7×7 @ 25mm 对称阵）：BoardConfig + 可复用检测器

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace calib {

enum class PreviewMode { Auto, Manual, None };

inline PreviewMode parse_preview(const std::string & s)
{
  if (s == "manual") return PreviewMode::Manual;
  if (s == "none") return PreviewMode::None;
  return PreviewMode::Auto;
}

inline std::string cli_str(cv::CommandLineParser & cli, const char * key)
{
  auto v = cli.get<std::string>(key);
  if (v == "none" || v == "true") return {};
  return v;
}

struct BoardConfig
{
  int pattern_cols = 7;
  int pattern_rows = 7;
  float center_distance_mm = 25.f;
  std::vector<double> R_gimbal2imubody = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  Eigen::Vector3d p_yaw_to_pitch_mm = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_pitch_to_barrel_mm = Eigen::Vector3d::Zero();

  float blob_min_area = 10.f;
  float blob_max_area = 3000.f;
  float blob_min_circularity = 0.6f;
  int blob_color = 0;       // 白底黑点
  float detect_scale = 0.5f;  // 预览检测缩放；保存/解算用 1.0
  int jpeg_quality = 92;
  double reproj_max_px = 1.0;

  cv::Size pattern_size() const { return {pattern_cols, pattern_rows}; }
};

inline Eigen::Vector3d read_vec3_mm(const YAML::Node & yaml, const char * key)
{
  if (!yaml[key]) return Eigen::Vector3d::Zero();
  auto v = yaml[key].as<std::vector<double>>();
  if (v.size() != 3) return Eigen::Vector3d::Zero();
  return {v[0], v[1], v[2]};
}

inline BoardConfig load_board_config(const std::string & path)
{
  BoardConfig c;
  auto yaml = YAML::LoadFile(path);
  if (yaml["pattern_cols"]) c.pattern_cols = yaml["pattern_cols"].as<int>();
  if (yaml["pattern_rows"]) c.pattern_rows = yaml["pattern_rows"].as<int>();
  if (yaml["center_distance_mm"])
    c.center_distance_mm = static_cast<float>(yaml["center_distance_mm"].as<double>());
  if (yaml["R_gimbal2imubody"])
    c.R_gimbal2imubody = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  c.p_yaw_to_pitch_mm = read_vec3_mm(yaml, "p_yaw_to_pitch_mm");
  c.p_pitch_to_barrel_mm = read_vec3_mm(yaml, "p_pitch_to_barrel_mm");
  if (yaml["blob_min_area"]) c.blob_min_area = yaml["blob_min_area"].as<float>();
  if (yaml["blob_max_area"]) c.blob_max_area = yaml["blob_max_area"].as<float>();
  if (yaml["blob_min_circularity"])
    c.blob_min_circularity = yaml["blob_min_circularity"].as<float>();
  if (yaml["blob_color"]) c.blob_color = yaml["blob_color"].as<int>();
  if (yaml["detect_scale"]) c.detect_scale = yaml["detect_scale"].as<float>();
  if (yaml["jpeg_quality"]) c.jpeg_quality = yaml["jpeg_quality"].as<int>();
  if (yaml["reproj_max_px"]) c.reproj_max_px = yaml["reproj_max_px"].as<double>();
  return c;
}

// blob 只建一次；灰度缓冲复用；可选降采样检测后再把点乘回全分辨率
class CircleGridDetector
{
public:
  explicit CircleGridDetector(const BoardConfig & cfg) : cfg_(cfg)
  {
    cv::SimpleBlobDetector::Params p;
    p.filterByArea = true;
    p.minArea = cfg.blob_min_area;
    p.maxArea = cfg.blob_max_area;
    p.filterByCircularity = true;
    p.minCircularity = cfg.blob_min_circularity;
    p.filterByInertia = true;
    p.minInertiaRatio = 0.3f;
    p.filterByConvexity = true;
    p.minConvexity = 0.5f;
    p.filterByColor = true;
    p.blobColor = cfg.blob_color;
    blob_ = cv::SimpleBlobDetector::create(p);
  }

  // full_res=true：全分辨率（保存复检 / 离线解算）
  bool detect(const cv::Mat & bgr_or_gray, std::vector<cv::Point2f> & centers, bool full_res = true)
  {
    centers.clear();
    if (bgr_or_gray.empty()) return false;

    if (bgr_or_gray.channels() == 1) {
      gray_ = bgr_or_gray;
    } else {
      cv::cvtColor(bgr_or_gray, gray_, cv::COLOR_BGR2GRAY);
    }

    float scale = full_res ? 1.f : cfg_.detect_scale;
    if (scale <= 0.f || scale > 1.f) scale = 1.f;

    const cv::Mat * src = &gray_;
    if (scale < 1.f) {
      cv::resize(gray_, scaled_, cv::Size(), scale, scale, cv::INTER_AREA);
      src = &scaled_;
    }

    bool ok = cv::findCirclesGrid(
      *src, cfg_.pattern_size(), centers, cv::CALIB_CB_SYMMETRIC_GRID, blob_);
    if (ok && scale < 1.f) {
      const float inv = 1.f / scale;
      for (auto & pt : centers) {
        pt.x *= inv;
        pt.y *= inv;
      }
    }
    return ok;
  }

  const BoardConfig & config() const { return cfg_; }

private:
  BoardConfig cfg_;
  cv::Ptr<cv::SimpleBlobDetector> blob_;
  cv::Mat gray_;
  cv::Mat scaled_;
};

// 平面板：角点原点左上（内参 / calibrateHandEye）
inline std::vector<cv::Point3f> centers_3d_planar(const BoardConfig & c)
{
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<size_t>(c.pattern_cols * c.pattern_rows));
  for (int i = 0; i < c.pattern_rows; i++) {
    for (int j = 0; j < c.pattern_cols; j++) {
      points.push_back({j * c.center_distance_mm, i * c.center_distance_mm, 0});
    }
  }
  return points;
}

// 竖直板：原点板心，Y 右、Z 上（robotworld / 英雄约定）
inline std::vector<cv::Point3f> centers_3d_vertical(const BoardConfig & c)
{
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<size_t>(c.pattern_cols * c.pattern_rows));
  for (int i = 0; i < c.pattern_rows; i++) {
    for (int j = 0; j < c.pattern_cols; j++) {
      float y = (j - 0.5f * (c.pattern_cols - 1)) * c.center_distance_mm;
      float z = -(i - 0.5f * (c.pattern_rows - 1)) * c.center_distance_mm;
      points.push_back({0.f, y, z});
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

inline bool load_intrinsics(
  const std::string & path, cv::Mat & camera_matrix, cv::Mat & distort_coeffs,
  std::string * err = nullptr)
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
  if (!camera_info_path.empty()) return load_intrinsics(camera_info_path, K, D, err);
  return load_intrinsics(config_path, K, D, err);
}

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

inline void show_preview(
  PreviewMode mode, const cv::Mat & bgr, const cv::Size & pattern,
  const std::vector<cv::Point2f> & centers, bool ok, const std::string & title)
{
  if (mode == PreviewMode::None) return;
  cv::Mat drawing;
  const float preview_scale = 0.5f;
  cv::resize(bgr, drawing, cv::Size(), preview_scale, preview_scale, cv::INTER_AREA);
  std::vector<cv::Point2f> scaled = centers;
  for (auto & p : scaled) {
    p.x *= preview_scale;
    p.y *= preview_scale;
  }
  cv::drawChessboardCorners(drawing, pattern, scaled, ok);
  cv::imshow(title, drawing);
  if (mode == PreviewMode::Manual) {
    cv::waitKey(0);
  } else {
    cv::waitKey(1);
  }
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
