#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <tuple>
#include <optional>

#include "tasks/auto_aim/yolos/yolov5.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
namespace multithread
{

class MultiThreadDetector
{
public:
  MultiThreadDetector(const std::string & config_path, bool debug);

  void push(cv::Mat img, std::chrono::steady_clock::time_point t);
  void close() { queue_.close(); }

  std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop();  //暂时不支持yolov8

  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop();
  std::optional<std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>>
    debug_pop_for(std::chrono::milliseconds timeout);

private:
  using Frame = std::tuple<cv::Mat, std::chrono::steady_clock::time_point, ov::InferRequest>;
  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> process(Frame frame);
  cv::Rect roi_;
  cv::Point2f offset_;
  bool use_roi_;
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  std::string device_;
  YOLO yolo_;

  tools::ThreadSafeQueue<std::tuple<cv::Mat, std::chrono::steady_clock::time_point, ov::InferRequest>> queue_{
    16, [] { tools::logger()->debug("[MultiThreadDetector] queue is full!"); }};
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP
