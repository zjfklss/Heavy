#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "thread_safe_queue.hpp"
namespace tools
{
class Recorder
{
public:
  // Recorder(double fps = 30);
  Recorder(int mode, double fps = 30);
  ~Recorder();
  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const rclcpp::Time & timestamp);
  void record(
    const cv::Mat & img,
    const rclcpp::Time & timestamp);
  void record(
    const Eigen::Quaterniond & q,
    const rclcpp::Time & timestamp);

private:
  struct FrameData
  {
    cv::Mat img;
    Eigen::Quaterniond q;
    rclcpp::Time timestamp;
  };
  struct FrameData_img
  {
    cv::Mat img;
    rclcpp::Time timestamp;
  };
  
  struct FrameData_q
  {
    Eigen::Quaterniond q;
    rclcpp::Time timestamp;
  };

  bool init_;
  std::atomic<bool> stop_thread_;
  double fps_;
  std::string text_path_;
  std::string video_path_;
  std::ofstream text_writer_;
  cv::VideoWriter video_writer_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  ThreadSafeQueue<FrameData> queue_;
  ThreadSafeQueue<FrameData_img> queue_img;
  ThreadSafeQueue<FrameData_q> queue_q;
  std::thread saving_thread_;  // 负责保存帧数据的线程
  std::thread saving_thread_img; 
  std::thread saving_thread_q; 
  void init(const cv::Mat & img);

  void init();

  void save_to_file();
  void save_to_file_img();
  void save_to_file_q();
  int mode_;  // 0: img + q, 1: only img, 2: only q


  //rclcpp::Clock steady_clock_{RCL_ROS_TIME};
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP