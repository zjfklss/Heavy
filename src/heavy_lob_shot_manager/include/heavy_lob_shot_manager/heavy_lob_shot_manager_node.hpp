#ifndef HEAVY_LOB_SHOT_MANAGER__HEAVY_LOB_SHOT_MANAGER_NODE_HPP_
#define HEAVY_LOB_SHOT_MANAGER__HEAVY_LOB_SHOT_MANAGER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/vector3.hpp"
#include "lob_shot_aiming/lob_shot_aiming.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace heavy_lob_shot_manager
{

enum class State
{
  IDLE,
  INHIBITED,
  WAITING_TF,
  YAW_AIMING,
  YAW_CONVERGING,
  PITCH_AIMING,
  PITCH_CONVERGING,
  READY
};

class HeavyLobShotManagerNode : public rclcpp::Node
{
public:
  explicit HeavyLobShotManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tick();
  void handleIdle();
  void handleInhibited();
  void handleWaitingTf();
  void handleYawAiming();
  void handleYawConverging();
  void handlePitchAiming();
  void handlePitchConverging();
  void handleReady();

  void setState(State s);
  std::string stateName(State s) const;
  bool canAim() const;
  bool yawConverged() const;
  bool pitchConverged() const;
  void publishStatus();
  void publishReady(bool ready);
  void publishGimbal(double pitch_up, double yaw_rel, uint8_t z);
  void publishTrajectory();
  void publishVisualization();
  void publishFieldMarkers();
  void clearVisualization();
  bool trajectoryHitsWindmill(double mx, double my, double mz) const;

  void onTrigger(const std_msgs::msg::Bool::SharedPtr msg);
  void onFire(const std_msgs::msg::Bool::SharedPtr msg);
  void onUnlocked(const std_msgs::msg::Bool::SharedPtr msg);
  void onDocked(const std_msgs::msg::Bool::SharedPtr msg);
  void onAssembling(const std_msgs::msg::Bool::SharedPtr msg);
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<lob_shot_aiming::LobShotAiming> aiming_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fire_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr unlocked_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr docked_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr assembling_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  State state_{State::IDLE};
  bool triggered_{false};
  bool cancel_{false};
  bool fire_{false};
  bool unlocked_{false};
  bool docked_{false};
  bool assembling_{false};
  bool joint_ok_{false};
  rclcpp::Time state_enter_;

  double current_yaw_{0.0};
  double current_pitch_{0.0};
  double target_yaw_{0.0};
  double target_yaw_world_{0.0};
  double target_pitch_up_{0.0};
  double last_horizontal_{0.0};

  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double armor_top_z_{1.721};
  double armor_bot_z_{1.214};
  double bullet_speed_{16.5};
  double friction_{0.0};
  double yaw_tolerance_{0.02};
  double pitch_tolerance_{0.02};
  double tf_timeout_{5.0};
  double aim_timeout_{5.0};
  double pitch_sign_{1.0};
  double yaw_sign_{1.0};
  double pitch_max_deg_{42.0};
  std::string feedback_mode_{"open_loop"};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};
  std::string muzzle_frame_{"muzzle"};
  std::string pitch_joint_name_{"pitch_joint"};
  std::string yaw_joint_name_{"yaw_joint"};

  double windmill_x_{14.5};
  double windmill_y_{8.0};
  double windmill_radius_{1.7};
  double windmill_z_min_{0.2};
  double windmill_z_max_{2.8};
  bool traj_blocked_{false};
};

}  // namespace heavy_lob_shot_manager

#endif  // HEAVY_LOB_SHOT_MANAGER__HEAVY_LOB_SHOT_MANAGER_NODE_HPP_
