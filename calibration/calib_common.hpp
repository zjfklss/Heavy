#pragma once
// 标定板配置与检测器：对称圆点阵（小板 7×7 @ 25mm）和棋盘格（大板内角点 11×8 @ 35mm）

#include <algorithm>
#include <atomic>
#include <thread>
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

enum class PatternType { Circles, Chessboard };

struct BoardConfig
{
  PatternType pattern_type = PatternType::Circles;
  // 圆点板数圆点；棋盘格数内角点（不是方格数）
  int pattern_cols = 7;
  int pattern_rows = 7;
  // 相邻特征点间距：圆点板为圆心距，棋盘格为方格边长（读 square_size_mm）
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
  bool chessboard() const { return pattern_type == PatternType::Chessboard; }
  std::string describe() const
  {
    return fmt::format(
      "{} {}x{} @ {:.1f}mm", chessboard() ? "棋盘格(内角点)" : "对称圆点板", pattern_cols, pattern_rows,
      center_distance_mm);
  }
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
  const auto type = yaml["pattern_type"] ? yaml["pattern_type"].as<std::string>() : "circles";
  if (type == "chessboard") {
    c.pattern_type = PatternType::Chessboard;
    if (!yaml["square_size_mm"]) throw std::runtime_error("chessboard 需要 square_size_mm（方格边长）");
  } else if (type != "circles") {
    throw std::runtime_error("pattern_type 只支持 circles 或 chessboard: " + type);
  }
  if (yaml["pattern_cols"]) c.pattern_cols = yaml["pattern_cols"].as<int>();
  if (yaml["pattern_rows"]) c.pattern_rows = yaml["pattern_rows"].as<int>();
  if (c.chessboard())
    c.center_distance_mm = static_cast<float>(yaml["square_size_mm"].as<double>());
  else if (yaml["center_distance_mm"])
    c.center_distance_mm = static_cast<float>(yaml["center_distance_mm"].as<double>());
  if (c.pattern_cols < 2 || c.pattern_rows < 2 || !(c.center_distance_mm > 0))
    throw std::runtime_error("标定板行列数需 >=2 且间距 >0");
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
class BoardDetector
{
public:
  explicit BoardDetector(const BoardConfig & cfg) : cfg_(cfg)
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
    // 预览降采样后，面积阈值按面积比缩小，物理尺寸与全分辨率一致
    const float s = (cfg.detect_scale > 0.f && cfg.detect_scale < 1.f) ? cfg.detect_scale : 1.f;
    p.minArea = cfg.blob_min_area * s * s;
    p.maxArea = cfg.blob_max_area * s * s;
    blob_preview_ = cv::SimpleBlobDetector::create(p);
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

    bool ok;
    if (cfg_.chessboard()) {
      // SB 检测自带亚像素精度；全分辨率用 ACCURACY，预览只求快
      const int flags = full_res ? cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY : cv::CALIB_CB_NORMALIZE_IMAGE;
      ok = cv::findChessboardCornersSB(*src, cfg_.pattern_size(), centers, flags);
    } else {
      auto blob = (scale < 1.f) ? blob_preview_ : blob_;
      ok = cv::findCirclesGrid(*src, cfg_.pattern_size(), centers, cv::CALIB_CB_SYMMETRIC_GRID, blob);
    }
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
  cv::Ptr<cv::SimpleBlobDetector> blob_preview_;
  cv::Mat gray_;
  cv::Mat scaled_;
};
using CircleGridDetector = BoardDetector;  // 旧名

// 一张输入图的全分辨率检测结果。图片按 1.jpg、2.jpg… 连续编号，遇到第一张缺失即停止。
struct Detection
{
  int index = 0;
  std::string path;
  cv::Size size;
  bool ok = false;
  std::vector<cv::Point2f> points;
};

// 多线程检测整个目录：每线程一个 BoardDetector（内部缓冲不可共享），结果按编号返回。
// 180° 点序对齐依赖前一张，仍由调用方顺序处理。
inline std::vector<Detection> detect_folder(
  const std::string & folder, const BoardConfig & board, unsigned threads = 0)
{
  std::vector<Detection> result;
  for (int i = 1;; i++) {
    auto path = fmt::format("{}/{}.jpg", folder, i);
    std::ifstream probe(path, std::ios::binary);
    if (!probe) break;
    result.push_back({i, path, {}, false, {}});
  }
  if (threads == 0) threads = std::max(1u, std::thread::hardware_concurrency());
  threads = std::min<unsigned>(threads, std::max<size_t>(1, result.size()));
  std::atomic<size_t> next{0};
  std::vector<std::thread> workers;
  for (unsigned t = 0; t < threads; t++)
    workers.emplace_back([&] {
      BoardDetector detector(board);
      for (size_t k; (k = next++) < result.size();) {
        auto & d = result[k];
        const auto image = cv::imread(d.path, cv::IMREAD_GRAYSCALE);
        if (image.empty()) continue;
        d.size = image.size();
        d.ok = detector.detect(image, d.points, true);
      }
    });
  for (auto & worker : workers) worker.join();
  // 与旧行为一致：遇到第一张无法解码的图片就截断
  for (size_t k = 0; k < result.size(); k++)
    if (result[k].size.empty()) {
      result.resize(k);
      break;
    }
  return result;
}

// 对称圆点阵 / 棋盘格 180° 翻转：与上一帧点序更近则整序反转。两序都离得很远时保持检测序。
inline bool align_symmetric_180(
  const std::vector<cv::Point2f> * prev, std::vector<cv::Point2f> & cur, double max_jump_px = 80.0)
{
  if (prev == nullptr || prev->size() != cur.size() || cur.empty()) return false;
  auto mean = [](const std::vector<cv::Point2f> & a, const std::vector<cv::Point2f> & b) {
    double s = 0;
    for (size_t i = 0; i < a.size(); i++) s += cv::norm(a[i] - b[i]);
    return s / static_cast<double>(a.size());
  };
  std::vector<cv::Point2f> flipped(cur.rbegin(), cur.rend());
  const double d0 = mean(*prev, cur);
  const double d1 = mean(*prev, flipped);
  (void)max_jump_px;
  // 姿势差很大时两序的平均位移都会很大，仍选明显更近的那一序
  if (d1 + 15.0 < d0) {
    cur.swap(flipped);
    return true;
  }
  return false;
}

// 采图四元数是 Rz(yaw)*R(-Y, joint_pitch)。eulers(2,1,0) 的 pitch 与关节角反号。
inline Eigen::Vector3d joint_from_gimbal_euler(const Eigen::Vector3d & ypr_rad)
{
  return {ypr_rad[0], -ypr_rad[1], ypr_rad[2]};
}

// 平面板：角点原点左上。只用于内参。两套手眼都用竖直板，结果才能对照。
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

// N.txt：一行 "w x y z"，云台系到世界系的四元数（与 hero::attitude(yaw, pitch) 相同约定）
inline bool read_q(const std::string & q_path, Eigen::Quaterniond & q)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  if (!(q_file >> w >> x >> y >> z)) return false;
  q = Eigen::Quaterniond(w, x, y, z);
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1) > 1e-3) return false;
  q.normalize();
  return true;
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

    if (yaml["source"] && yaml["source"].as<std::string>() == "remote_hero_untuned") {
      fmt::print(stderr, "警告: 这是远程英雄旧相机内参，本车重标定前不要用来解手眼\n");
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

// hero.yaml 的 camera_rpy、ROS launch_params 的 rpy：URDF 相机本体系（X 前 Y 左 Z 上）到云台的 Z-Y-X 欧拉角，
// 返回 [roll, pitch, yaw]（rad）。OpenCV 光学系 = 本体系 · R_gimbal2ideal^T。
inline Eigen::Vector3d camera_rpy_urdf(const Eigen::Matrix3d & R_camera2gimbal)
{
  const Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  const Eigen::Matrix3d B = R_camera2gimbal * R_gimbal2ideal;
  return {std::atan2(B(2, 1), B(2, 2)), -std::asin(std::clamp(B(2, 0), -1.0, 1.0)), std::atan2(B(1, 0), B(0, 0))};
}

// 一组手眼样本：云台位姿（世界系，mm）、PnP 得到的板到相机位姿（mm）、检测点。
struct HandEyeSample
{
  std::string path;
  Eigen::Matrix3d R_gimbal2world;
  Eigen::Vector3d t_gimbal2world_mm;
  Eigen::Matrix3d R_board2camera;
  Eigen::Vector3d t_board2camera_mm;
  std::vector<cv::Point2f> points;
};

struct ChainCheck
{
  double mean_px = 0, max_px = 0;
  std::string worst;
  Eigen::Matrix3d R_board2world;
  Eigen::Vector3d t_board2world_mm;
};

// 手眼结果的自洽检查：板在世界中固定不动，用结果把每张图串起来——
//   板(世界) → 世界到云台（串口姿态 + 轴偏置）→ 云台到相机（标定结果）→ 像素，
// 与检测点比较。board_pose 为空时，用各样本推出的板位姿取平均（适用于只解出相机外参的方法）。
inline ChainCheck check_chain(
  const std::vector<HandEyeSample> & samples, const std::vector<cv::Point3f> & board_points,
  const Eigen::Matrix3d & R_camera2gimbal, const Eigen::Vector3d & t_camera2gimbal_mm, const cv::Mat & K,
  const cv::Mat & D, const Eigen::Matrix3d * R_board2world = nullptr, const Eigen::Vector3d * t_board2world = nullptr)
{
  ChainCheck check;
  if (R_board2world && t_board2world) {
    check.R_board2world = *R_board2world;
    check.t_board2world_mm = *t_board2world;
  } else {
    Eigen::Vector4d q_sum = Eigen::Vector4d::Zero();
    Eigen::Vector3d t_sum = Eigen::Vector3d::Zero();
    for (const auto & s : samples) {
      const Eigen::Matrix3d R_c2w = s.R_gimbal2world * R_camera2gimbal;
      const Eigen::Vector3d t_c2w = s.R_gimbal2world * t_camera2gimbal_mm + s.t_gimbal2world_mm;
      Eigen::Quaterniond q(R_c2w * s.R_board2camera);
      if (q_sum.dot(q.coeffs()) < 0) q.coeffs() *= -1;
      q_sum += q.coeffs();
      t_sum += R_c2w * s.t_board2camera_mm + t_c2w;
    }
    check.R_board2world = Eigen::Quaterniond(q_sum.normalized()).toRotationMatrix();
    check.t_board2world_mm = t_sum / std::max<size_t>(1, samples.size());
  }
  double total = 0;
  size_t count = 0;
  for (const auto & s : samples) {
    const Eigen::Matrix3d R_c2w = s.R_gimbal2world * R_camera2gimbal;
    const Eigen::Vector3d t_c2w = s.R_gimbal2world * t_camera2gimbal_mm + s.t_gimbal2world_mm;
    const Eigen::Matrix3d R_b2c = R_c2w.transpose() * check.R_board2world;
    const Eigen::Vector3d t_b2c = R_c2w.transpose() * (check.t_board2world_mm - t_c2w);
    cv::Mat R_cv, rvec, tvec;
    cv::eigen2cv(R_b2c, R_cv);
    cv::Rodrigues(R_cv, rvec);
    cv::eigen2cv(t_b2c, tvec);
    std::vector<cv::Point2f> projected;
    cv::projectPoints(board_points, rvec, tvec, K, D, projected);
    double sample_total = 0;
    for (size_t i = 0; i < projected.size(); i++) sample_total += cv::norm(projected[i] - s.points[i]);
    const double sample_mean = sample_total / std::max<size_t>(1, projected.size());
    if (sample_mean > check.max_px) {
      check.max_px = sample_mean;
      check.worst = s.path;
    }
    total += sample_total;
    count += projected.size();
  }
  check.mean_px = count ? total / count : 0;
  return check;
}

inline void print_chain(const ChainCheck & check)
{
  fmt::print(
    "链路重投影（板→世界→云台→相机→像素）: 平均 {:.3f} px，最差一张 {:.3f} px ({})\n"
    "  经验：平均 < 1 px 可用；> 3 px 检查姿态同步、云台轴偏置、板是否被碰动\n",
    check.mean_px, check.max_px, check.worst);
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
