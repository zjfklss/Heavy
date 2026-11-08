// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "jlcv_serial_driver/jlcv_serial_driver.hpp"

// ROS
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include "serial_driver/serial_driver.hpp"
#include "std_msgs/msg/string.hpp"

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "jlcv_serial_driver/crc.hpp"
#include "jlcv_serial_driver/packet.hpp"

namespace jlcv_serial_driver
{
SerialDriver::SerialDriver(const rclcpp::NodeOptions& options)
  : Node("jlcv_serial_driver", options)
  , owned_ctx_{ new IoContext(2) }
  , serial_driver_{ new drivers::serial_driver::SerialDriver(*owned_ctx_) }
  , deploy_mode_enabled_(false)                // 默认关闭
  , last_deploy_msg_time_(this->now())        // 初始化时间
  , DEPLOY_TIMEOUT_SECONDS_(0.1)              // 超时 200ms
{
  RCLCPP_INFO(get_logger(), "Start SerialDriver!!");

   // 初始化前哨站状态标志为 false
  last_outpost_status_ = false;

  // whether or not sentry robot  该参数用于判断是否为哨兵机器人
  debug_ = this->declare_parameter("debug", false);

  // 从参数服务器获取串口相关参数并进行配置
  getParams();

  // offset time
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);

  // Create Publisher
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));
  game_status_pub_ = this->create_publisher<jlcv_interfaces::msg::GameStatus>("/game_status", rclcpp::QoS(rclcpp::KeepLast(1)));
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);

  deploy_mode_enabled_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/from_custom_client", 10, std::bind(&SerialDriver::DeployModeEnalbedCallback, this, std::placeholders:: _1)
  );


  deploy_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SerialDriver::check_timeout, this));


  // Detect parameter client
  armor_detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try
  {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open())
    {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&SerialDriver::receiveData, this);
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }
  
  target_picth_yaw_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
     "/gimbal_command", rclcpp::SensorDataQoS(),
     std::bind(&SerialDriver::AngleSendData, this, std::placeholders::_1));
}

SerialDriver::~SerialDriver()
{
  if (receive_thread_.joinable())
  {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open())
  {
    serial_driver_->port()->close();
  }

  if (owned_ctx_)
  {
    owned_ctx_->waitForExit();
  }
}

//
void SerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok())
  {
    try
    {
      serial_driver_->port()->receive(header);
      //std::cout << "header[0] : " << header[0] << std::endl;
      if (header[0] == 0x5A)
      {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);

        bool crc_ok = crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
        if (crc_ok)
        {
          if (!initial_set_param_ || packet.enemy_color != previous_receive_color_)
          {
            ResetColor(rclcpp::Parameter("enemy_color", int(packet.enemy_color)));
            previous_receive_color_ = packet.enemy_color;
          }

          if (packet.reset_tracker)
          {
            ResetTracker();
          }

          // update outpost status
          last_outpost_status_ = curr_outpost_status_;
          curr_outpost_status_ = packet.outpost_status;
          if(last_outpost_status_ == false && curr_outpost_status_ == true)
          {
            auto detector_object_param = rclcpp::Parameter("ignore_classes", std::vector<std::string>{"negative"});
            ResetDetectorObject(detector_object_param);
          }
          else if(last_outpost_status_ == true && curr_outpost_status_ == false)
          {
            auto detector_object_param = rclcpp::Parameter("ignore_classes", std::vector<std::string>{"negative", "guard"});
            ResetDetectorObject(detector_object_param);
          }

          sensor_msgs::msg::JointState joint_state;
          jlcv_interfaces::msg::GameStatus gamestatus;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          joint_state.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          joint_state.name.push_back("pitch_joint");
          joint_state.name.push_back("yaw_joint");
          joint_state.position.push_back(packet.curr_pitch);
          joint_state.position.push_back(packet.curr_yaw);
          gamestatus.enemy_color = packet.enemy_color;
          gamestatus.game_status = packet.game_status;
          gamestatus.outpost_status = packet.outpost_status;

          gamestatus.camera_status = 1 - packet.camera_status;//选择相机是长焦还是短焦 0-短焦，1-长焦
          gamestatus.bullet_speed = packet.bullet_speed;

          joint_state_pub_->publish(joint_state);
          game_status_pub_->publish(gamestatus);
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      }
      else
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void SerialDriver::AngleSendData(geometry_msgs::msg::Vector3::SharedPtr target_angle)
{
  msg_mutex.lock();
  try
  {
    SendPacket sendpacket;
    SendPacketAngle packet;
    packet.x = target_angle->x;
    packet.y = target_angle->y;
    packet.z = target_angle->z;
    packet.deploy_mode_enabled = deploy_mode_enabled_ ? 1 : 0;;

    // RCLCPP_INFO(get_logger(),"deploy_mode_enabled: %d", packet.deploy_mode_enabled);

    sendpacket.header = SEND_HEADER_ANGLE;
    sendpacket.data.packetangle = packet;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&sendpacket), sizeof(sendpacket));
    std::vector<uint8_t> data = toVector<SendPacket>(sendpacket);
    serial_driver_->port()->send(data);

    if (debug_)
    {
      // std::cout << "fire" << packet.z << std::endl;
      // std::cout << "pitch = " << packet.x << std::endl << "yaw = " << packet.y << std::endl;
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
  msg_mutex.unlock();
}

void SerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try
  {
    device_name_ = declare_parameter<std::string>("device_name", "");
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try
  {
    baud_rate = declare_parameter<int>("baud_rate", 921600);
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try
  {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none")
    {
      fc = FlowControl::NONE;
    }
    else if (fc_string == "hardware")
    {
      fc = FlowControl::HARDWARE;
    }
    else if (fc_string == "software")
    {
      fc = FlowControl::SOFTWARE;
    }
    else
    {
      throw std::invalid_argument{ "The flow_control parameter must be one of: none, software, or hardware." };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try
  {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none")
    {
      pt = Parity::NONE;
    }
    else if (pt_string == "odd")
    {
      pt = Parity::ODD;
    }
    else if (pt_string == "even")
    {
      pt = Parity::EVEN;
    }
    else
    {
      throw std::invalid_argument{ "The parity parameter must be one of: none, odd, or even." };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try
  {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0")
    {
      sb = StopBits::ONE;
    }
    else if (sb_string == "1.5")
    {
      sb = StopBits::ONE_POINT_FIVE;
    }
    else if (sb_string == "2" || sb_string == "2.0")
    {
      sb = StopBits::TWO;
    }
    else
    {
      throw std::invalid_argument{ "The stop_bits parameter must be one of: 1, 1.5, or 2." };
    }
  }
  catch (rclcpp::ParameterTypeException& ex)
  {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }
  // std::cout << "baud_rate: " << baud_rate << std::endl;
  device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void SerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try
  {
    if (serial_driver_->port()->is_open())
    {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok())
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void SerialDriver::ResetColor(const rclcpp::Parameter& param)
{
  if (!armor_detector_param_client_->service_is_ready())
  {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  // Set armor enemy_color
  if (!set_param_future_.valid() || set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    RCLCPP_INFO(get_logger(), "Setting armor enemy_color to %s...", param.as_int() == 0 ? "RED" : "BLUE");
    armor_detector_param_client_->set_parameters({ param });
    initial_set_param_ = true;
  }
}

void SerialDriver::ResetDetectorObject(const rclcpp::Parameter& param)
{
  if (!armor_detector_param_client_->service_is_ready())
  {
    RCLCPP_WARN(get_logger(), "Armor Detector Service not ready, skipping parameter set");
    return;
  }

  if (!set_param_future_.valid() || set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    RCLCPP_INFO(get_logger(), "Finished ResetDetectorObject");
    armor_detector_param_client_->set_parameters({ param });
  }
}

void SerialDriver::ResetTracker()
{
  if (!reset_tracker_client_->service_is_ready())
  {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

void SerialDriver::DeployModeEnalbedCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  
  // 收到话题 → deploy_mode_enabled 置 1
  deploy_mode_enabled_ = true;
  
  // 更新最后接收时间
  last_deploy_msg_time_ = this->now();

  // RCLCPP_INFO(get_logger(), "Received deploy command, set deploy_mode_enabled = 1");
}
void SerialDriver::check_timeout()
{
  std::lock_guard<std::mutex> lock(msg_mutex_);

  // 计算当前时间与最后一次接收消息的时间差
  double time_diff = (this->now() - last_deploy_msg_time_).seconds();

  // 超时 → 恢复为 0
  if (time_diff > DEPLOY_TIMEOUT_SECONDS_)
  {
    deploy_mode_enabled_ = false;
  }

  // RCLCPP_INFO(get_logger(), "Received deploy no command, set deploy_mode_enabled = 0");
}

}  // namespace jlcv_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(jlcv_serial_driver::SerialDriver)