#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "MvCameraControl.h"
#include "io/camera.hpp"
#include "settings.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
// 海康 USB3 工业相机封装：内部以守护线程监控断流并做 USB 复位重连，
// 采集线程写入单元素覆盖队列（queue_(1)），保证 read() 总是取最新一帧（200 ms 超时返回空图）。
class HikRobot : public CameraBase
{
public:
  HikRobot(
    double exposure_ms, double gain, const std::string & vid_pid, double fps = 150,
    const HikRobotSettings & settings = {});
  ~HikRobot() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  uint64_t frames_captured() const override { return frames_captured_; }

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  double exposure_us_;
  double gain_;
  double fps_;
  HikRobotSettings settings_;

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_;

  void * handle_ = nullptr;
  std::thread capture_thread_;
  std::atomic<bool> capturing_{false};
  std::atomic<bool> capture_quit_{false};
  tools::ThreadSafeQueue<CameraData, true> queue_;

  int vid_, pid_;
  std::atomic<uint64_t> frames_captured_{0};

  void capture_start();
  void capture_stop();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, unsigned int value);
  bool set_int_value(const std::string & name, int value);
  bool set_bool_value(const std::string & name, bool value);

  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP
