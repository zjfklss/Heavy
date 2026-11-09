#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace tools::jlcv_show {
enum Curve { command_yaw, feedback_yaw, command_pitch, feedback_pitch, command_shoot, curve_count };
struct Point { double time=0,value=0; bool valid=true; };
struct ArmorPose {
  Eigen::Vector4d xyza=Eigen::Vector4d::Zero();
  double width=.135,height=.125;
  // Solver's fixed plate Euler pitch. Its local +X points inward, opposite the receiving face.
  double pitch=15*3.14159265358979323846/180;
};
Eigen::Vector3d plate_normal(const ArmorPose&);
Eigen::Vector3d plate_up(const ArmorPose&);
struct Snapshot {
  std::array<std::vector<Point>,curve_count> curves;
  std::vector<ArmorPose> armors;
  std::optional<Eigen::Vector3d> aim;
  cv::Mat preview;
  std::string source="等待数据",status="正在启动",target;
  double time=0,feedback_age=-1,estimate_age=-1;
  bool control=false,shoot=false;
};
struct Range { double minimum,maximum; };
Range value_range(const std::vector<Point>& a,const std::vector<Point>& b,double since,
                  double minimum_span=.4);

// Bounded diagnostic mailbox. Vision/transport publish here; only the main Qt thread paints.
class Channel {
 public:
  static constexpr size_t maximum_points=4096;
  static constexpr double history_seconds=10;
  void reset(std::string source);
  void status(std::string message);
  void feedback(double time,double yaw,double pitch);
  void command(double time,double yaw,double pitch,bool control,bool shoot);
  void estimate(std::vector<ArmorPose> armors,std::optional<Eigen::Vector3d> aim,
                std::string target,const cv::Mat& image);
  Snapshot snapshot() const;
  bool stopping() const { return stopping_.load(); }
  void stop() { stopping_=true; }
 private:
  using Clock=std::chrono::steady_clock;
  mutable std::mutex mutex_;
  std::array<std::deque<Point>,curve_count> curves_;
  Snapshot latest_;
  Clock::time_point feedback_at_{},estimate_at_{},preview_at_{};
  std::optional<double> sent_yaw_,returned_yaw_;
  std::atomic<bool> stopping_{false};
  void append(Curve,double time,double value,bool valid=true);
};
// QApplication and all widgets live on the caller's main thread. Worker owns vision and I/O.
int run_dashboard(const std::function<int(Channel&)>& worker);
}
