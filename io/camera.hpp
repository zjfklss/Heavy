#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
// 相机工厂：yaml camera_name 选海康/迈德威视。read() 给 BGR 图 + 取图时刻（steady_clock 软时间戳）。
// 这个时刻会拿去 HeroBoard::imu_at 插值云台姿态，下游别改写它。
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  // 运行时调曝光/增益（旧 ROS camera_node 参数回调的等价物）；不支持的相机忽略
  virtual void set_exposure_gain(double /*exposure_ms*/, double /*gain*/) {}
};

class Camera
{
public:
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  void set_exposure_gain(double exposure_ms, double gain);

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP