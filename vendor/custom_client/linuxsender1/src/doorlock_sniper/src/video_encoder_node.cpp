#include "doorlock_sniper/video_encoder_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <sstream>

namespace doorlock_sniper
{

VideoEncoderNode::VideoEncoderNode(const rclcpp::NodeOptions & options)
: Node("video_encoder_node", options)
{
  param_input_topic_ = this->declare_parameter("input_topic", "/image_raw");
  param_crop_size_ = this->declare_parameter("crop_size", 800);
  param_output_size_ = this->declare_parameter("output_size", 400);
  param_output_fps_ = this->declare_parameter("output_fps", 60);
  param_static_simplify_ = this->declare_parameter("static_simplify", true);
  param_motion_threshold_ = this->declare_parameter("motion_threshold", 14);
  param_motion_erode_px_ = this->declare_parameter("motion_erode_px", 1);
  param_motion_dilate_px_ = this->declare_parameter("motion_dilate_px", 2);
  param_bg_update_alpha_ = this->declare_parameter("bg_update_alpha", 0.01);
  param_bg_blur_sigma_ = this->declare_parameter("bg_blur_sigma", 1.2);
  param_center_clear_size_ = this->declare_parameter("center_clear_size", 100);
  param_force_monochrome_ = this->declare_parameter("force_monochrome", false);
  param_enable_display_ = this->declare_parameter("enable_display", true);
  param_webp_quality_ = this->declare_parameter("webp_quality", 3);
  param_encode_width_ = this->declare_parameter("encode_width", 120);
  param_encode_height_ = this->declare_parameter("encode_height", 120);

  serial_device_ = this->declare_parameter("serial_device", "/dev/ttyUSB0");
  serial_baud_rate_ = this->declare_parameter("serial_baud_rate", 921600);
  serial_send_hz_ = this->declare_parameter("serial_send_hz", 50);

  if (param_output_fps_ < 1) param_output_fps_ = 1;
  if (param_output_fps_ > 60) param_output_fps_ = 60;
  if (serial_send_hz_ < 1) serial_send_hz_ = 1;
  if (serial_send_hz_ > 50) serial_send_hz_ = 50;
  if (param_webp_quality_ < 1) param_webp_quality_ = 1;
  if (param_webp_quality_ > 100) param_webp_quality_ = 100;
  if (param_encode_width_ < 16) param_encode_width_ = 16;
  if (param_encode_height_ < 16) param_encode_height_ = 16;

  RCLCPP_INFO(this->get_logger(), "Initializing serial sender: %s @ %d baud",
              serial_device_.c_str(), serial_baud_rate_);

  try {
    auto serial_options = rclcpp::NodeOptions()
      .append_parameter_override("device_name", serial_device_)
      .append_parameter_override("baud_rate", serial_baud_rate_)
      .append_parameter_override("debug", false);

    serial_sender_ = std::make_shared<serial_test::SerialTest>(serial_options);
    serial_sender_->setFromClientCallback(
      [this](const std::string & data) { handle_custom_client_data(data); });
    RCLCPP_INFO(this->get_logger(), "Serial sender initialized");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial: %s", e.what());
    throw;
  }

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    param_input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&VideoEncoderNode::image_callback, this, std::placeholders::_1));

  int timer_interval_ms = 1000 / serial_send_hz_;
  serial_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_interval_ms),
    std::bind(&VideoEncoderNode::serial_timer_callback, this));

  if (param_enable_display_) {
    display_running_ = true;
    display_thread_ = std::thread(&VideoEncoderNode::display_loop, this);
  }

  double serial_throughput_bps = static_cast<double>(param_max_serial_payload_) * serial_send_hz_;
  double per_frame_budget = serial_throughput_bps / std::max(param_output_fps_, 1);
  RCLCPP_INFO(this->get_logger(),
    "VideoEncoderNode started: crop=%d -> encode=%dx%d q=%d @ %dfps, "
    "serial=%s@%d %dHz, budget=%.0f B/frame, throughput=%.0f B/s",
    param_crop_size_, param_encode_width_, param_encode_height_,
    param_webp_quality_, param_output_fps_,
    serial_device_.c_str(), serial_baud_rate_, serial_send_hz_,
    per_frame_budget, serial_throughput_bps);
}

VideoEncoderNode::~VideoEncoderNode()
{
  if (param_enable_display_) {
    display_running_ = false;
    if (display_thread_.joinable()) display_thread_.join();
    cv::destroyAllWindows();
  }
}

cv::Mat VideoEncoderNode::preprocess_image(
  const cv::Mat & input,
  cv::Mat * roi_downsample,
  cv::Mat * static_removed)
{
  int x = (input.cols - param_crop_size_) / 2;
  int y = (input.rows - param_crop_size_) / 2;
  x = std::max(0, x);
  y = std::max(0, y);
  int w = std::min(param_crop_size_, input.cols - x);
  int h = std::min(param_crop_size_, input.rows - y);

  cv::Mat cropped = input(cv::Rect(x, y, w, h));
  cv::Mat resized;
  cv::resize(cropped, resized, cv::Size(param_output_size_, param_output_size_),
             0, 0, cv::INTER_LINEAR);
  if (roi_downsample) {
    resized.copyTo(*roi_downsample);
  }
  cv::Mat working = resized;
  if (param_force_monochrome_) {
    cv::Mat gray_full;
    cv::cvtColor(working, gray_full, cv::COLOR_BGR2GRAY);
    cv::cvtColor(gray_full, working, cv::COLOR_GRAY2BGR);
  }

  if (!param_static_simplify_) {
    if (static_removed) {
      working.copyTo(*static_removed);
    }
    return working;
  }

  cv::Mat gray;
  cv::cvtColor(working, gray, cv::COLOR_BGR2GRAY);
  if (background_gray_f32_.empty()) {
    gray.convertTo(background_gray_f32_, CV_32F);
    return working;
  }

  cv::Mat bg_u8;
  cv::convertScaleAbs(background_gray_f32_, bg_u8);

  cv::Mat diff;
  cv::absdiff(gray, bg_u8, diff);

  cv::Mat motion_mask;
  cv::threshold(diff, motion_mask, param_motion_threshold_, 255, cv::THRESH_BINARY);
  if (param_motion_erode_px_ > 0) {
    if (motion_erode_kernel_.empty()) {
      const int k = 2 * param_motion_erode_px_ + 1;
      motion_erode_kernel_ = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(k, k));
    }
    cv::erode(motion_mask, motion_mask, motion_erode_kernel_, cv::Point(-1, -1), 1);
  }
  if (param_motion_dilate_px_ > 0) {
    if (motion_dilate_kernel_.empty()) {
      const int k = 2 * param_motion_dilate_px_ + 1;
      motion_dilate_kernel_ = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(k, k));
    }
    cv::dilate(motion_mask, motion_mask, motion_dilate_kernel_, cv::Point(-1, -1), 1);
  }
  if (param_center_clear_size_ > 0) {
    const int clear_size = std::min({param_center_clear_size_, working.cols, working.rows});
    const int x0 = std::max(0, working.cols / 2 - clear_size / 2);
    const int y0 = std::max(0, working.rows / 2 - clear_size / 2);
    const int cw = std::min(clear_size, working.cols - x0);
    const int ch = std::min(clear_size, working.rows - y0);
    cv::rectangle(motion_mask, cv::Rect(x0, y0, cw, ch), cv::Scalar(255), cv::FILLED);
  }

  cv::Mat static_base = working.clone();
  cv::Mat blurred_static;
  cv::GaussianBlur(
    static_base,
    blurred_static,
    cv::Size(),
    std::max(0.0, param_bg_blur_sigma_),
    std::max(0.0, param_bg_blur_sigma_));

  cv::Mat focused = blurred_static.clone();
  working.copyTo(focused, motion_mask);
  if (static_removed) {
    focused.copyTo(*static_removed);
  }

  cv::accumulateWeighted(gray, background_gray_f32_, std::clamp(param_bg_update_alpha_, 0.001, 0.2));
  return focused;
}

void VideoEncoderNode::encode_frame_webp(const cv::Mat & frame, cv::Mat * debug_output)
{
  cv::Mat encode_input;
  cv::resize(frame, encode_input,
    cv::Size(param_encode_width_, param_encode_height_), 0, 0, cv::INTER_AREA);

  std::vector<uint8_t> webp_buf;
  std::vector<int> encode_params = {cv::IMWRITE_WEBP_QUALITY, param_webp_quality_};
  if (!cv::imencode(".webp", encode_input, webp_buf, encode_params)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "WebP encode failed");
    return;
  }

  if (webp_buf.size() > 65535) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "WebP frame too large: %zu bytes, dropping", webp_buf.size());
    return;
  }

  EncodedFrame ef;
  ef.data.reserve(4 + webp_buf.size());
  uint32_t total_size = static_cast<uint32_t>(webp_buf.size());
  ef.data.push_back(static_cast<uint8_t>(total_size & 0xFF));
  ef.data.push_back(static_cast<uint8_t>((total_size >> 8) & 0xFF));
  ef.data.push_back(static_cast<uint8_t>((total_size >> 16) & 0xFF));
  ef.data.push_back(static_cast<uint8_t>((total_size >> 24) & 0xFF));
  ef.data.insert(ef.data.end(), webp_buf.begin(), webp_buf.end());

  stat_encoded_frames_++;
  stat_encoded_bytes_total_ += webp_buf.size();
  if (webp_buf.size() > stat_max_frame_size_) stat_max_frame_size_ = webp_buf.size();
  if (webp_buf.size() < stat_min_frame_size_) stat_min_frame_size_ = webp_buf.size();

  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    total_queued_bytes_ += ef.data.size();
    frame_queue_.push_back(std::move(ef));

    size_t queue_byte_limit = param_max_serial_payload_ * serial_send_hz_;
    while (total_queued_bytes_ > queue_byte_limit && frame_queue_.size() > 1) {
      auto & oldest = frame_queue_.front();
      total_queued_bytes_ -= oldest.data.size();
      frame_queue_.pop_front();
      stat_drop_frames_++;
    }
  }
}

void VideoEncoderNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    if (!deploy_mode_enabled_.load()) {
      return;
    }

    if (param_output_fps_ < 60) {
      const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
      const int64_t frame_interval_ns = 1000000000LL / std::max(param_output_fps_, 1);
      const int64_t now_ns = (stamp_ns > 0) ? stamp_ns : this->now().nanoseconds();
      if (last_encode_stamp_ns_ > 0 && (now_ns - last_encode_stamp_ns_) < frame_interval_ns) {
        return;
      }
      last_encode_stamp_ns_ = now_ns;
    }

    cv::Mat input = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat roi_downsample;
    cv::Mat static_removed;
    cv::Mat processed = preprocess_image(input, &roi_downsample, &static_removed);

    if (param_enable_display_) {
      cv::Mat raw_preview;
      cv::resize(input, raw_preview,
        cv::Size(std::max(1, input.cols / 2), std::max(1, input.rows / 2)),
        0, 0, cv::INTER_AREA);
      std::lock_guard<std::mutex> lock(frame_mutex_);
      raw_preview.copyTo(display_raw_frame_);
      roi_downsample.copyTo(display_roi_frame_);
      static_removed.copyTo(display_static_frame_);
      processed.copyTo(display_frame_);
    }

    encode_frame_webp(processed, nullptr);
    frame_count_++;

  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "image_callback error: %s", e.what());
  }
}

void VideoEncoderNode::serial_timer_callback()
{
  if (!deploy_mode_enabled_.load()) {
    clear_encoded_buffer();
    sending_active_ = false;
    sending_data_.clear();
    sending_offset_ = 0;
    return;
  }

  if (!sending_active_) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (frame_queue_.empty()) return;

    auto & front = frame_queue_.front();
    sending_data_ = std::move(front.data);
    sending_offset_ = 0;
    sending_active_ = true;
    total_queued_bytes_ -= front.data.size();
    frame_queue_.pop_front();
  }

  size_t remaining = sending_data_.size() - sending_offset_;
  if (remaining == 0) {
    sending_active_ = false;
    sending_data_.clear();
    return;
  }

  size_t chunk_len = std::min(remaining, param_max_serial_payload_);
  std::vector<uint8_t> serial_buf(300, 0);
  serial_buf[0] = static_cast<uint8_t>(chunk_len & 0xFF);
  serial_buf[1] = static_cast<uint8_t>((chunk_len >> 8) & 0xFF);
  std::memcpy(serial_buf.data() + 2, sending_data_.data() + sending_offset_, chunk_len);

  serial_sender_->sendToCustomClient(serial_buf.data(), 300);
  sending_offset_ += chunk_len;
  serial_bytes_sent_ += chunk_len;
  serial_packets_sent_++;

  if (sending_offset_ >= sending_data_.size()) {
    sending_active_ = false;
    sending_data_.clear();
  }

  static auto last_print = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 1) {
    size_t avg_frame = stat_encoded_frames_ > 0
      ? stat_encoded_bytes_total_ / stat_encoded_frames_ : 0;
    float packets_per_frame = stat_encoded_frames_ > 0
      ? static_cast<float>(serial_packets_sent_) / stat_encoded_frames_ : 0;

    RCLCPP_INFO(this->get_logger(),
      "TX: %lu pkt/s %lu B/s | Enc: %lu frm/s avg=%zu B pkt/frm=%.1f | "
      "qlen=%zu drop=%lu",
      serial_packets_sent_, serial_bytes_sent_,
      stat_encoded_frames_, avg_frame, packets_per_frame,
      frame_queue_.size(), stat_drop_frames_);

    serial_packets_sent_ = 0;
    serial_bytes_sent_ = 0;
    stat_encoded_frames_ = 0;
    stat_encoded_bytes_total_ = 0;
    stat_max_frame_size_ = 0;
    stat_min_frame_size_ = SIZE_MAX;
    stat_drop_frames_ = 0;
    last_print = now;
  }
}

void VideoEncoderNode::handle_custom_client_data(const std::string & data)
{
  if (data.size() < 2) return;

  const uint16_t value = (static_cast<uint8_t>(data[0]) << 8) |
    static_cast<uint8_t>(data[1]);
  const bool enabled = (value == 1024);
  const bool previous = deploy_mode_enabled_.exchange(enabled);

  if (previous == enabled) return;

  if (!enabled) {
    clear_encoded_buffer();
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Deploy mode %s, video streaming %s",
    enabled ? "enabled" : "disabled",
    enabled ? "started" : "stopped");
}

void VideoEncoderNode::clear_encoded_buffer()
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  frame_queue_.clear();
  total_queued_bytes_ = 0;
}

void VideoEncoderNode::display_loop()
{
  cv::namedWindow("Doorlock Sniper Raw", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper ROI", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper Static", cv::WINDOW_NORMAL);
  cv::namedWindow("Doorlock Sniper", cv::WINDOW_NORMAL);

  while (display_running_ && rclcpp::ok()) {
    cv::Mat raw_frame, roi_frame, static_frame, frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!display_raw_frame_.empty()) display_raw_frame_.copyTo(raw_frame);
      if (!display_roi_frame_.empty()) display_roi_frame_.copyTo(roi_frame);
      if (!display_static_frame_.empty()) display_static_frame_.copyTo(static_frame);
      if (!display_frame_.empty()) display_frame_.copyTo(frame);
    }

    if (!raw_frame.empty()) cv::imshow("Doorlock Sniper Raw", raw_frame);
    if (!roi_frame.empty()) cv::imshow("Doorlock Sniper ROI", roi_frame);
    if (!static_frame.empty()) cv::imshow("Doorlock Sniper Static", static_frame);
    if (!frame.empty()) cv::imshow("Doorlock Sniper", frame);

    cv::waitKey(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }

  cv::destroyAllWindows();
}

} // namespace doorlock_sniper

RCLCPP_COMPONENTS_REGISTER_NODE(doorlock_sniper::VideoEncoderNode)
