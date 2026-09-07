#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
constexpr double kPitchMin = 10.0 * M_PI / 180.0;
constexpr double kPitchMax = 42.0 * M_PI / 180.0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("heavy_lob_dry_run_watch");

  bool ready = false;
  double pitch = 0.0;
  double z = 0.0;
  bool got_cmd = false;

  auto status_sub = node->create_subscription<std_msgs::msg::String>(
    "/lob_shot/status", 10,
    [&](const std_msgs::msg::String::SharedPtr msg) {
      if (msg->data == "READY") {
        ready = true;
      }
    });

  auto cmd_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
    "/gimbal_command", rclcpp::SensorDataQoS(),
    [&](const geometry_msgs::msg::Vector3::SharedPtr msg) {
      pitch = msg->x;
      z = msg->z;
      got_cmd = true;
    });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  rclcpp::Rate rate(50);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    if (ready && got_cmd && std::fabs(pitch) >= kPitchMin &&
      std::fabs(pitch) <= kPitchMax + 0.02 && z >= 0.5 && z < 1.5)
    {
      RCLCPP_INFO(
        node->get_logger(),
        "DRY_RUN PASS pitch=%.2f deg z=%.0f", pitch * 180.0 / M_PI, z);
      rclcpp::shutdown();
      return 0;
    }
    rate.sleep();
  }

  RCLCPP_ERROR(
    node->get_logger(),
    "DRY_RUN FAIL ready=%d got_cmd=%d pitch=%.3f z=%.1f",
    static_cast<int>(ready), static_cast<int>(got_cmd), pitch, z);
  rclcpp::shutdown();
  return 1;
}
