#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <optional>
#include <tuple>

#include "tasks/auto_aim/yolos/yolov5.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
namespace multithread
{

// 流水线推理：采集线程 push（letterbox + start_async 立即返回），主线程 pop 时才 wait。
// GPU 同时算多帧，吞吐不再被单帧推理耗时卡死。输出布局按 yolov5 解析（yolo_name 需为 yolov5）。
class MultiThreadDetector
{
public:
  // max_inflight：在途帧上限。延迟≈max_inflight/推理吞吐，满了丢新帧；同济原值 16
  MultiThreadDetector(const std::string & config_path, bool debug = false, size_t max_inflight = 16);

  void push(cv::Mat img, std::chrono::steady_clock::time_point t);

  std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop();  //暂时不支持yolov8

  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop();

  // 限时版：无帧时返回空，主循环才能响应 Ctrl+C 和切模式
  std::optional<std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>>
  debug_pop_for(std::chrono::milliseconds timeout);

  const std::string & device() const { return device_; }
  int dropped() const { return dropped_.load(); }

private:
  // input 必须随请求一起存活：start_async 只拿了它的指针
  using Job = std::tuple<cv::Mat, std::chrono::steady_clock::time_point, ov::InferRequest, cv::Mat>;

  ov::Core core_;
  ov::CompiledModel compiled_model_;
  std::string device_;
  YOLO yolo_;
  std::atomic<int> dropped_{0};

  tools::ThreadSafeQueue<Job> queue_;

  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> finish(Job & job);
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP
