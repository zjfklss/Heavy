#include "rclcpp/rclcpp.hpp"
#include "heavy_lob_shot_manager/heavy_lob_shot_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<heavy_lob_shot_manager::HeavyLobShotManagerNode>());
  rclcpp::shutdown();
  return 0;
}
