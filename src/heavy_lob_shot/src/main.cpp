#include "rclcpp/rclcpp.hpp"
#include "heavy_lob_shot/manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<heavy_lob_shot::ManagerNode>());
  rclcpp::shutdown();
  return 0;
}
