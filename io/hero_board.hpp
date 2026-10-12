#ifndef IO__HERO_BOARD_HPP_
#define IO__HERO_BOARD_HPP_

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "tasks/lob_shot/lob_serial.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{

// 自家英雄 jlcv_serial_driver：0x5A 回传，0xA5 的 x=pitch y=yaw z=0/1/2。
// 调用形状照 CBoard，供同济主循环使用。
class HeroBoard
{
public:
  double bullet_speed;
  Mode mode;

  explicit HeroBoard(const std::string & config_path);
  ~HeroBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  void send(Command command) const;

  void latest_joint(double & yaw, double & pitch) const;
  int camera_id() const { return camera_id_.load(); }  // 0 短焦上置，1 长焦下置
  bool take_reset();

private:
  struct Sample
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  static Eigen::Quaterniond joint_to_q(double yaw, double pitch);
  static Mode parse_mode(const std::string & name);
  void rx_loop();

  tools::ThreadSafeQueue<Sample> queue_;
  Sample data_ahead_;
  Sample data_behind_;

  std::unique_ptr<lob_shot::SerialSender> serial_;
  bool deploy_ = false;
  Mode yaml_mode_ = Mode::auto_aim;
  std::atomic<double> joint_yaw_{0};
  std::atomic<double> joint_pitch_{0};
  std::atomic<int> camera_id_{0};
  std::atomic<bool> reset_pending_{false};
  std::atomic<bool> rx_run_{false};
  std::thread rx_;
};

}  // namespace io

#endif
