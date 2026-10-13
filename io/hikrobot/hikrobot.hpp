#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "MvCameraControl.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
// 海康 USB3 相机。守护线程管断线重连（reset_usb + 重开），抓图线程 Bayer→BGR 后入队。
// USB2 口只有 ~24fps，启动时会 warn；read() 最多等 100ms，没帧也回主循环，Ctrl+C 才能退。
class HikRobot : public CameraBase
{
public:
  HikRobot(double exposure_ms, double gain, const std::string & vid_pid);
  ~HikRobot() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  void set_exposure_gain(double exposure_ms, double gain) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  std::atomic<double> exposure_us_;
  std::atomic<double> gain_;
  bool speed_checked_ = false;

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_;

  void * handle_;
  std::thread capture_thread_;
  std::atomic<bool> capturing_;
  std::atomic<bool> capture_quit_;
  // 满了丢旧留新：消费者慢时也拿最新一帧，延迟不随积压增长
  tools::ThreadSafeQueue<CameraData, true> queue_;

  int vid_, pid_;

  void capture_start();
  void capture_stop();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, unsigned int value);

  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;
  void warn_if_usb2();
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP