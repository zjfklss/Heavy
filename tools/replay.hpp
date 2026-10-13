#ifndef TOOLS__REPLAY_HPP
#define TOOLS__REPLAY_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "tools/logger.hpp"

namespace tools
{

// 离线回放 tools::Recorder 录的 <path>.avi + <path>.txt（每行 t w x y z，与 tests/auto_aim_test 相同）。
// 按录制时间轴实时放：下游 Aimer 用 to_now 预测，快放会把延迟算错。
class Replay
{
public:
  // fast=true：不按时间轴、读完就给，只用来压测推理吞吐（预测量会失真）
  explicit Replay(const std::string & path, bool fast = false)
  : video_(path + ".avi"), text_(path + ".txt"), fast_(fast)
  {
    if (!video_.isOpened() || !text_) {
      logger()->error("[Replay] 打不开 {}.avi / {}.txt", path, path);
    } else {
      logger()->info("[Replay] {}", path);
    }
  }

  // 读完返回 false
  bool read(cv::Mat & img, std::chrono::steady_clock::time_point & t, Eigen::Quaterniond & q)
  {
    double ts, w, x, y, z;
    if (!video_.read(img) || !(text_ >> ts >> w >> x >> y >> z)) return false;

    const auto rel = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(ts));
    if (fast_) {
      t = std::chrono::steady_clock::now();
      q = Eigen::Quaterniond(w, x, y, z).normalized();
      return true;
    }
    if (!started_) {
      t0_ = std::chrono::steady_clock::now() - rel;
      started_ = true;
    } else if (rel - last_rel_ > std::chrono::milliseconds(500)) {
      // 录像中途暂停过（同济 demo 开头就空了 25s）：跳过空档，别真等
      t0_ -= rel - last_rel_ - std::chrono::milliseconds(10);
    }
    last_rel_ = rel;
    t = t0_ + rel;
    std::this_thread::sleep_until(t);
    q = Eigen::Quaterniond(w, x, y, z).normalized();
    return true;
  }

private:
  cv::VideoCapture video_;
  std::ifstream text_;
  bool fast_;
  std::chrono::steady_clock::time_point t0_;
  std::chrono::steady_clock::duration last_rel_{};
  bool started_ = false;
};

}  // namespace tools

#endif  // TOOLS__REPLAY_HPP
