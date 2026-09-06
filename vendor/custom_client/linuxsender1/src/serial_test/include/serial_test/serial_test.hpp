#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>

#include <thread>
#include <mutex>
#include <memory>
#include <string>
#include <vector>
#include <cstring>
#include <functional>

namespace serial_test
{

class SerialTest : public rclcpp::Node
{
public:
    explicit SerialTest(const rclcpp::NodeOptions& options);
    ~SerialTest();

    // 发送 0x0310 数据（机器人→自定义客户端）
    void sendToCustomClient(const uint8_t* data, size_t len);
    void sendToCustomClient(const std::string& data);
    void setFromClientCallback(std::function<void(const std::string&)> callback);

private:
    std::unique_ptr<drivers::common::IoContext> owned_ctx_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::string device_name_;
    std::thread receive_thread_;
    std::mutex send_mutex_;
    
    uint8_t packet_seq_ = 0;
    bool debug_;
    
    int send_count_ = 0;
    int last_count_ = 0;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr from_client_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr to_client_sub_;
    std::function<void(const std::string&)> from_client_callback_;
    
    void getParams();
    void receiveData();
    void send0310Packet(const uint8_t* data, size_t len);
    void send0311Packet(const uint8_t* data, size_t len);
    void printFrequency();
    void reopenPort();
};

} // namespace serial_test
