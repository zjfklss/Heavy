#ifndef DOORLOCK_SNIPER_VIDEO_ENCODER_NODE_HPP_
#define DOORLOCK_SNIPER_VIDEO_ENCODER_NODE_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <deque>
#include <memory>

#include "doorlock_sniper/msg/video_packet.hpp"
#include "serial_test/serial_test.hpp"

namespace doorlock_sniper
{

struct EncodedFrame {
  std::vector<uint8_t> data;
  size_t send_offset = 0;
};

class VideoEncoderNode : public rclcpp::Node
{
public:
  explicit VideoEncoderNode(const rclcpp::NodeOptions & options);
  ~VideoEncoderNode() override;

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  cv::Mat preprocess_image(
    const cv::Mat & input,
    cv::Mat * roi_downsample = nullptr,
    cv::Mat * static_removed = nullptr);

  void encode_frame_webp(const cv::Mat & frame, cv::Mat * debug_output);
  void serial_timer_callback();
  void handle_custom_client_data(const std::string & data);
  void clear_encoded_buffer();

  void display_loop();

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr serial_timer_;

  // Serial
  std::shared_ptr<serial_test::SerialTest> serial_sender_;

  // Stats
  uint32_t frame_count_ = 0;
  uint64_t serial_bytes_sent_ = 0;
  uint64_t serial_packets_sent_ = 0;
  uint64_t stat_encoded_frames_ = 0;
  size_t stat_encoded_bytes_total_ = 0;
  size_t stat_max_frame_size_ = 0;
  size_t stat_min_frame_size_ = SIZE_MAX;
  uint64_t stat_drop_frames_ = 0;

  // Display
  std::thread display_thread_;
  std::atomic<bool> display_running_{false};
  std::mutex frame_mutex_;
  cv::Mat display_raw_frame_;
  cv::Mat display_roi_frame_;
  cv::Mat display_static_frame_;
  cv::Mat display_frame_;

  // Frame queue (WebP-encoded frames for serial transmission)
  std::deque<EncodedFrame> frame_queue_;
  size_t total_queued_bytes_ = 0;
  std::mutex buffer_mutex_;
  std::atomic<bool> deploy_mode_enabled_{false};

  // Current frame being sent
  std::vector<uint8_t> sending_data_;
  size_t sending_offset_ = 0;
  bool sending_active_ = false;

  // Frame rate control
  int64_t last_encode_stamp_ns_ = 0;
  uint64_t display_frame_counter_ = 0;

  // Background subtraction
  cv::Mat background_gray_f32_;
  cv::Mat motion_erode_kernel_;
  cv::Mat motion_dilate_kernel_;

  // Parameters
  int param_crop_size_ = 800;
  int param_output_size_ = 400;
  int param_output_fps_ = 60;
  int param_target_bitrate_ = 90;
  bool param_static_simplify_ = true;
  int param_motion_threshold_ = 14;
  int param_motion_erode_px_ = 1;
  int param_motion_dilate_px_ = 2;
  double param_bg_update_alpha_ = 0.01;
  double param_bg_blur_sigma_ = 1.2;
  int param_center_clear_size_ = 100;
  bool param_force_monochrome_ = false;
  bool param_enable_display_ = true;
  bool param_debug_dump_enable_ = false;
  int param_debug_dump_every_n_frames_ = 20;
  bool param_debug_dump_save_raw_ = true;
  bool param_debug_dump_save_roi_ = true;
  bool param_debug_dump_save_static_ = true;
  bool param_debug_dump_save_final_ = true;
  std::string param_input_topic_;
  std::string param_x264_preset_ = "auto";
  std::string param_debug_dump_dir_ = "sniper_debug_imgs";

  // Serial parameters
  std::string serial_device_ = "/dev/ttyUSB0";
  int serial_baud_rate_ = 921600;
  int serial_send_hz_ = 50;

  // WebP encoding parameters (new)
  int param_webp_quality_ = 3;
  int param_encode_width_ = 120;
  int param_encode_height_ = 120;
  size_t param_max_serial_payload_ = 298;
};

} // namespace doorlock_sniper

#endif
