#include "tools/recorder.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <string>
#include "tools/ros_time.hpp"

namespace tools
{
// Recorder::Recorder(double fps) : init_(false), fps_(fps), queue_(1), stop_thread_(false), mode_(0)
// {
//   // start_time_ = g_ros_clock.now();
//   // last_time_ = start_time_;

//   auto folder_path = "records";
//   auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
//   text_path_ = fmt::format("{}/{}.txt", folder_path, file_name);
//   video_path_ = fmt::format("{}/{}.avi", folder_path, file_name);

//   std::filesystem::create_directory(folder_path);
// }

Recorder::Recorder(int mode, double fps) : init_(false), fps_(fps), queue_(1), queue_img(1),queue_q(1), stop_thread_(false), mode_(mode)
{
  // start_time_ = g_ros_clock.now();
  // last_time_ = start_time_;

  auto folder_path = "records";
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  text_path_ = fmt::format("{}/{}.txt", folder_path, file_name);
  video_path_ = fmt::format("{}/{}.avi", folder_path, file_name);

  std::filesystem::create_directory(folder_path);
}


Recorder::~Recorder()
{
  stop_thread_ = true;
  // 退出时给队列中额外推入一个空帧，避免pop一直等待
  queue_.push({cv::Mat::zeros(0, 0, 0), {0, 0, 0, 0}, last_time_});
  queue_img.push({cv::Mat::zeros(0, 0, 0), last_time_});
  queue_q.push({{0, 0, 0, 0}, last_time_});
  if (saving_thread_.joinable()) saving_thread_.join();  // 等待视频保存线程结束

  if (!init_) return;
  text_writer_.close();
  if(mode_==0 || mode_==1)
    video_writer_.release();
}

void Recorder::save_to_file()
{
  while (!stop_thread_) {
    FrameData frame;
    queue_.pop(frame);  // 从队列中取出帧数据
    if (frame.img.empty()) {
      //tools::logger()->debug("Recorder received empty img. Skip this frame.");
      continue;
    }
    // 写入视频文件
    video_writer_.write(frame.img);

    // 写入文本文件（输出顺序为wxyz）
    Eigen::Vector4d xyzw = frame.q.coeffs();
    auto since_begin = delta_time(frame.timestamp, start_time_);
    text_writer_ << fmt::format(
      "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  }
}
void Recorder::save_to_file_img()
{
  while (!stop_thread_) {
    FrameData_img frame;
    queue_img.pop(frame);  // 从队列中取出帧数据
    if (frame.img.empty()) {
      //tools::logger()->debug("Recorder received empty img. Skip this frame.");
      continue;
    }
    // 写入视频文件
    video_writer_.write(frame.img);
  }
}

void Recorder::save_to_file_q()
{
  while (!stop_thread_) {
    FrameData_q frame;
    if(queue_q.empty())
    {
      //tools::logger()->debug("Recorder received empty q. Skip this frame.");
      continue;
    } 
    queue_q.pop(frame);  // 从队列中取出帧数据

    // 写入文本文件（输出顺序为wxyz）
    Eigen::Vector4d xyzw = frame.q.coeffs();
    auto since_begin = delta_time(frame.timestamp, start_time_);
    text_writer_ <<since_begin<<" "<<xyzw[3]<<" "<<xyzw[0]<<" "<<xyzw[1]<<" "<<xyzw[2]<<"\n";
  }
}

void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const rclcpp::Time & timestamp)
{
  if (img.empty()) return;
  if (!init_)
  {
    start_time_ = timestamp;
    
    last_time_ = start_time_;
    init(img);
  } 

  auto since_last = delta_time(timestamp, start_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_.push({img, q, timestamp});
}

void Recorder::record(
  const cv::Mat & img,
  const rclcpp::Time & timestamp)
{
  if (img.empty()) return;
  if (!init_)
  {
    start_time_ = timestamp;
    
    last_time_ = start_time_;
    init(img);
  } 

  auto since_last = delta_time(timestamp, start_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_img.push({img, timestamp});
}

void Recorder::record(
 const Eigen::Quaterniond & q,
  const rclcpp::Time & timestamp)
{
  if (!init_)
  {
    start_time_ = timestamp;
    
    last_time_ = start_time_;
    init();
  } 
  auto since_last = delta_time(timestamp, start_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_q.push({q, timestamp});
}

void Recorder::init(const cv::Mat & img)
{
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img.size());
  if(mode_==0)
  {
    text_writer_.open(text_path_);
    saving_thread_ = std::thread(&Recorder::save_to_file, this);  // 启动保存线程
  }else if(mode_==1)
  {
    saving_thread_ = std::thread(&Recorder::save_to_file_img, this);  // 启动保存线程
  }else if(mode_==2)  {
    saving_thread_ = std::thread(&Recorder::save_to_file_q, this);}

  init_ = true;
}

void Recorder::init()
{
  text_writer_.open(text_path_);
  saving_thread_ = std::thread(&Recorder::save_to_file_q, this);  // 启动保存线程
  init_ = true;
}


}  // namespace tools
