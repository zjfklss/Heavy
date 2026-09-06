// src/main.cpp
#include "rclcpp/rclcpp.hpp"
#include "serial_test/serial_test.hpp"
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serial_test::SerialTest>(rclcpp::NodeOptions());
    
    RCLCPP_INFO(node->get_logger(), "Starting 50Hz data transmission...");
    
    // 使用定时器控制发送频率：50Hz = 20ms 间隔
    rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
        std::chrono::milliseconds(20),  // 20ms = 50Hz
        [node]() {
            static int count = 0;
            std::string msg = "Robot d" + std::to_string(count++) + " ";
            node->sendToCustomClient(msg);
        }
    );
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}