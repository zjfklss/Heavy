// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "serial_driver/serial_driver.hpp"

// ROS2 message publish
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "jlcv_interfaces/msg/game_status.hpp"
#include "std_msgs/msg/string.hpp"

// ROS2 message subscribe
#include "jlcv_interfaces/msg/target.hpp"
#include "jlcv_interfaces/srv/reset_color.hpp"

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <future>

namespace jlcv_serial_driver
{
class SerialDriver : public rclcpp::Node
{
public:
  explicit SerialDriver(const rclcpp::NodeOptions& options);
  ~SerialDriver() override;

private:
  void getParams();

  void receiveData();

  void AngleSendData(geometry_msgs::msg::Vector3::SharedPtr target_angle);

  void reopenPort();

  void ResetColor(const rclcpp::Parameter& param);

  void ResetDetectorObject(const rclcpp::Parameter& param);

  void ResetTracker();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::thread receive_thread_;
  std::thread send_thread_;
  std::mutex msg_mutex;

  double timestamp_offset_ = 0;

  // Service client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  // Param client to set detect_colr
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  bool previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr armor_detector_param_client_;
  ResultFuturePtr set_param_future_;
  bool last_outpost_status_;
  bool curr_outpost_status_;

  // message_pub
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<jlcv_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // message_sub
  rclcpp::Subscription<jlcv_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_picth_yaw_sub_;

  // 部署模式相关
  std::mutex msg_mutex_;                          // 线程安全锁
  bool deploy_mode_enabled_;                      // 核心标志位
  rclcpp::Time last_deploy_msg_time_;             // 最后一次收到话题的时间
  const double DEPLOY_TIMEOUT_SECONDS_ = 0.1;      // 超时时间 200ms
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr deploy_mode_enabled_sub_;
  rclcpp::TimerBase::SharedPtr deploy_timer_;
  void DeployModeEnalbedCallback(const std_msgs::msg::String::SharedPtr msg);
  void check_timeout();




  // debug
  bool debug_;
};
}  // namespace jlcv_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
