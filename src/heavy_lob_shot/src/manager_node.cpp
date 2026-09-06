#include "heavy_lob_shot/manager_node.hpp"
#include "heavy_lob_shot/windmill.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "projectile_motion/gaf_projectile_solver.hpp"
#include "tf2/exceptions.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace heavy_lob_shot
{

namespace
{
constexpr uint8_t kAim = 1;
constexpr uint8_t kFire = 2;
}  // namespace

ManagerNode::ManagerNode(const rclcpp::NodeOptions & options)
: Node("heavy_lob_manager", options)
{
  // 默认打蓝方基地中间偏平大装甲。换算见 docs/修改说明.md（2026 V2.2.0 图 4-5/4-10/4-11/4-12）。
  target_x_ = declare_parameter("target_x", 25.603);
  target_y_ = declare_parameter("target_y", 8.000);
  target_z_ = declare_parameter("target_z", 1.602);
  armor_top_z_ = declare_parameter("armor_top_z", 1.721);
  armor_bot_z_ = declare_parameter("armor_bot_z", 1.214);
  bullet_speed_ = declare_parameter("bullet_speed", 16.5);
  friction_ = declare_parameter("friction_coeff", 0.0);
  declare_parameter("gravity", 9.7913);
  yaw_tolerance_ = declare_parameter("yaw_tolerance", 0.02);
  pitch_tolerance_ = declare_parameter("pitch_tolerance", 0.02);
  tf_timeout_ = declare_parameter("tf_timeout", 5.0);
  aim_timeout_ = declare_parameter("aim_timeout", 5.0);
  pitch_sign_ = declare_parameter("pitch_sign", 1.0);
  yaw_sign_ = declare_parameter("yaw_sign", 1.0);
  pitch_max_deg_ = declare_parameter("pitch_max_deg", 42.0);
  feedback_mode_ = declare_parameter("feedback_mode", std::string("open_loop"));
  map_frame_ = declare_parameter("map_frame", std::string("map"));
  base_frame_ = declare_parameter("base_frame", std::string("base_link"));
  muzzle_frame_ = declare_parameter("muzzle_frame", std::string("muzzle"));
  pitch_joint_name_ = declare_parameter("pitch_joint_name", std::string("pitch_joint"));
  yaw_joint_name_ = declare_parameter("yaw_joint_name", std::string("yaw_joint"));
  declare_parameter("muzzle_xyz", std::vector<double>{0.0, 0.0, 0.40});
  unlocked_ = declare_parameter("module_unlocked", false);
  docked_ = declare_parameter("module_docked", false);
  assembling_ = declare_parameter("module_assembling", false);
  windmill_x_ = declare_parameter("windmill_x", 14.5);
  windmill_y_ = declare_parameter("windmill_y", 8.0);
  windmill_radius_ = declare_parameter("windmill_radius", 1.7);
  windmill_z_min_ = declare_parameter("windmill_z_min", 0.2);
  windmill_z_max_ = declare_parameter("windmill_z_max", 2.8);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  aiming_ = std::make_shared<lob_shot_aiming::LobShotAiming>(
    tf_buffer_, map_frame_, base_frame_, muzzle_frame_);

  trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/heavy/lob_trigger", 10,
    std::bind(&ManagerNode::onTrigger, this, std::placeholders::_1));
  fire_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/heavy/fire", 10,
    std::bind(&ManagerNode::onFire, this, std::placeholders::_1));
  unlocked_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/heavy/module_unlocked", 10,
    std::bind(&ManagerNode::onUnlocked, this, std::placeholders::_1));
  docked_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/heavy/module_docked", 10,
    std::bind(&ManagerNode::onDocked, this, std::placeholders::_1));
  assembling_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/heavy/module_assembling", 10,
    std::bind(&ManagerNode::onAssembling, this, std::placeholders::_1));
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&ManagerNode::onJointState, this, std::placeholders::_1));

  gimbal_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
    "/gimbal_command", rclcpp::SensorDataQoS());
  status_pub_ = create_publisher<std_msgs::msg::String>("/heavy/status", 10);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>("/heavy/ready_to_shoot", 10);
  traj_pub_ = create_publisher<visualization_msgs::msg::Marker>("/heavy/trajectory", 10);
  viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/heavy/visualization", 10);

  state_enter_ = now();
  timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&ManagerNode::tick, this));

  RCLCPP_INFO(
    get_logger(),
    "heavy_lob_manager target=(%.3f, %.3f, %.3f) v=%.2f feedback=%s frames=%s->%s->%s",
    target_x_, target_y_, target_z_, bullet_speed_, feedback_mode_.c_str(),
    map_frame_.c_str(), base_frame_.c_str(), muzzle_frame_.c_str());
}

void ManagerNode::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  publishFieldMarkers();

  if (!canAim()) {
    if (state_ != State::INHIBITED) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "module gate closed (unlocked=%d docked=%d assembling=%d), inhibit gimbal",
        static_cast<int>(unlocked_), static_cast<int>(docked_), static_cast<int>(assembling_));
      triggered_ = false;
      fire_ = false;
      setState(State::INHIBITED);
    }
    handleInhibited();
    return;
  }

  if (cancel_) {
    cancel_ = false;
    triggered_ = false;
    fire_ = false;
    if (state_ != State::IDLE) {
      RCLCPP_INFO(get_logger(), "lob cancelled, release gimbal");
      setState(State::IDLE);
    }
  }

  if (state_ == State::INHIBITED) {
    setState(State::IDLE);
  }

  switch (state_) {
    case State::IDLE: handleIdle(); break;
    case State::INHIBITED: handleInhibited(); break;
    case State::WAITING_TF: handleWaitingTf(); break;
    case State::YAW_AIMING: handleYawAiming(); break;
    case State::YAW_CONVERGING: handleYawConverging(); break;
    case State::PITCH_AIMING: handlePitchAiming(); break;
    case State::PITCH_CONVERGING: handlePitchConverging(); break;
    case State::READY: handleReady(); break;
  }
  publishVisualization();
}

void ManagerNode::handleIdle()
{
  publishReady(false);
  publishStatus();
  clearVisualization();
  if (triggered_) {
    triggered_ = false;
    RCLCPP_INFO(get_logger(), "lob trigger, wait TF");
    setState(State::WAITING_TF);
  }
}

void ManagerNode::handleInhibited()
{
  publishReady(false);
  publishStatus();
}

void ManagerNode::handleWaitingTf()
{
  publishStatus();
  if (aiming_->isTfReady()) {
    setState(State::YAW_AIMING);
    return;
  }
  if ((now() - state_enter_).seconds() > tf_timeout_) {
    RCLCPP_ERROR(get_logger(), "TF timeout");
    setState(State::IDLE);
  }
}

void ManagerNode::handleYawAiming()
{
  const auto result = aiming_->solveYaw(target_x_, target_y_, target_z_);
  if (!result.success) {
    RCLCPP_ERROR(get_logger(), "%s", result.message.c_str());
    setState(State::IDLE);
    return;
  }
  target_yaw_ = result.angle;
  target_yaw_world_ = result.angle_world;
  RCLCPP_INFO(
    get_logger(), "yaw %.2f deg (map %.2f deg)",
    target_yaw_ * 180.0 / M_PI, result.angle_world * 180.0 / M_PI);
  setState(State::YAW_CONVERGING);
}

void ManagerNode::handleYawConverging()
{
  if (feedback_mode_ == "open_loop") {
    publishGimbal(0.0, target_yaw_, kAim);
  } else {
    geometry_msgs::msg::Vector3 cmd;
    cmd.x = current_pitch_;
    cmd.y = yaw_sign_ * target_yaw_;
    cmd.z = static_cast<double>(kAim);
    gimbal_pub_->publish(cmd);
  }
  publishStatus();
  if (yawConverged()) {
    setState(State::PITCH_AIMING);
    return;
  }
  if ((now() - state_enter_).seconds() > aim_timeout_) {
    RCLCPP_ERROR(get_logger(), "yaw converge timeout");
    setState(State::IDLE);
  }
}

void ManagerNode::handlePitchAiming()
{
  const auto result = aiming_->solvePitch(
    target_x_, target_y_, target_z_, bullet_speed_, friction_);
  if (!result.success) {
    RCLCPP_ERROR(get_logger(), "%s", result.message.c_str());
    setState(State::IDLE);
    return;
  }
  last_horizontal_ = result.horizontal_dist;
  const double pitch_max = pitch_max_deg_ * M_PI / 180.0;
  const double high = result.angle;
  const double low = result.low_pitch;
  if (high <= pitch_max + 1e-6) {
    target_pitch_up_ = high;
  } else if (low <= pitch_max + 1e-6) {
    target_pitch_up_ = low;
    RCLCPP_WARN(
      get_logger(),
      "high %.2f deg exceeds pitch_max %.1f deg, use low %.2f deg",
      high * 180.0 / M_PI, pitch_max_deg_, low * 180.0 / M_PI);
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "both ballistic roots exceed pitch_max %.1f deg (high %.2f low %.2f), inhibit",
      pitch_max_deg_, high * 180.0 / M_PI, low * 180.0 / M_PI);
    setState(State::IDLE);
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "pitch %.2f deg (high %.2f low %.2f max %.1f) dist=%.3f dh=%.3f",
    target_pitch_up_ * 180.0 / M_PI, high * 180.0 / M_PI, low * 180.0 / M_PI,
    pitch_max_deg_, result.horizontal_dist, result.height_diff);

  geometry_msgs::msg::TransformStamped tf_muzzle;
  try {
    tf_muzzle = tf_buffer_->lookupTransform(map_frame_, muzzle_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "TF map->muzzle failed: %s", ex.what());
    setState(State::IDLE);
    return;
  }
  traj_blocked_ = trajectoryHitsWindmill(
    tf_muzzle.transform.translation.x,
    tf_muzzle.transform.translation.y,
    tf_muzzle.transform.translation.z);
  publishTrajectory();
  if (traj_blocked_) {
    RCLCPP_ERROR(
      get_logger(),
      "trajectory hits center windmill (%.2f, %.2f) r=%.2f z=[%.2f,%.2f], inhibit",
      windmill_x_, windmill_y_, windmill_radius_, windmill_z_min_, windmill_z_max_);
    setState(State::IDLE);
    return;
  }
  setState(State::PITCH_CONVERGING);
}

void ManagerNode::handlePitchConverging()
{
  publishGimbal(target_pitch_up_, target_yaw_, kAim);
  publishStatus();
  if (pitchConverged()) {
    RCLCPP_INFO(get_logger(), "READY (not firing until /heavy/fire)");
    setState(State::READY);
    return;
  }
  if ((now() - state_enter_).seconds() > aim_timeout_) {
    RCLCPP_ERROR(get_logger(), "pitch converge timeout");
    setState(State::IDLE);
  }
}

void ManagerNode::handleReady()
{
  publishReady(true);
  const uint8_t z = fire_ ? kFire : kAim;
  publishGimbal(target_pitch_up_, target_yaw_, z);
  publishTrajectory();
  publishStatus();
  if (triggered_) {
    triggered_ = false;
    fire_ = false;
    setState(State::WAITING_TF);
  }
}

void ManagerNode::setState(State s)
{
  if (s == state_) {
    return;
  }
  RCLCPP_INFO(get_logger(), "state %s -> %s", stateName(state_).c_str(), stateName(s).c_str());
  state_ = s;
  state_enter_ = now();
}

std::string ManagerNode::stateName(State s) const
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::INHIBITED: return "INHIBITED";
    case State::WAITING_TF: return "WAITING_TF";
    case State::YAW_AIMING: return "YAW_AIMING";
    case State::YAW_CONVERGING: return "YAW_CONVERGING";
    case State::PITCH_AIMING: return "PITCH_AIMING";
    case State::PITCH_CONVERGING: return "PITCH_CONVERGING";
    case State::READY: return "READY";
  }
  return "UNKNOWN";
}

bool ManagerNode::canAim() const
{
  return unlocked_ && docked_ && !assembling_;
}

bool ManagerNode::yawConverged() const
{
  if (feedback_mode_ == "open_loop") {
    return true;
  }
  if (!joint_ok_) {
    return false;
  }
  double err = current_yaw_ - target_yaw_;
  while (err > M_PI) {
    err -= 2.0 * M_PI;
  }
  while (err < -M_PI) {
    err += 2.0 * M_PI;
  }
  return std::fabs(err) < yaw_tolerance_;
}

bool ManagerNode::pitchConverged() const
{
  if (feedback_mode_ == "open_loop") {
    return true;
  }
  if (!joint_ok_) {
    return false;
  }
  const double cmd_pitch = pitch_sign_ * target_pitch_up_;
  return std::fabs(current_pitch_ - cmd_pitch) < pitch_tolerance_;
}

void ManagerNode::publishStatus()
{
  std_msgs::msg::String msg;
  msg.data = stateName(state_);
  status_pub_->publish(msg);
}

void ManagerNode::publishReady(bool ready)
{
  std_msgs::msg::Bool msg;
  msg.data = ready;
  ready_pub_->publish(msg);
}

void ManagerNode::publishGimbal(double pitch_up, double yaw_rel, uint8_t z)
{
  geometry_msgs::msg::Vector3 cmd;
  cmd.x = pitch_sign_ * pitch_up;
  cmd.y = yaw_sign_ * yaw_rel;
  cmd.z = static_cast<double>(z);
  gimbal_pub_->publish(cmd);
}

void ManagerNode::publishTrajectory()
{
  if (last_horizontal_ < 0.2) {
    return;
  }
  geometry_msgs::msg::TransformStamped tf_muzzle;
  try {
    tf_muzzle = tf_buffer_->lookupTransform(map_frame_, muzzle_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;
  }
  const double mx = tf_muzzle.transform.translation.x;
  const double my = tf_muzzle.transform.translation.y;
  const double mz = tf_muzzle.transform.translation.z;
  const double dx = target_x_ - mx;
  const double dy = target_y_ - my;
  const double norm_xy = std::hypot(dx, dy);
  const double dir_x = (norm_xy > 0.01) ? dx / norm_xy : 1.0;
  const double dir_y = (norm_xy > 0.01) ? dy / norm_xy : 0.0;

  // 弹道采样抄 ITL GafProjectileSolver::computeTrajectory
  projectile_motion::GafProjectileSolver solver(bullet_speed_, friction_);
  const auto traj_2d = solver.computeTrajectory(target_pitch_up_, last_horizontal_, 60);

  visualization_msgs::msg::Marker mk;
  mk.header.stamp = now();
  mk.header.frame_id = map_frame_;
  mk.ns = "lob";
  mk.id = 1;
  mk.type = visualization_msgs::msg::Marker::LINE_STRIP;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = 0.06;
  mk.color.r = 1.0;
  mk.color.g = traj_blocked_ ? 0.08 : 0.45;
  mk.color.b = traj_blocked_ ? 0.08 : 0.0;
  mk.color.a = 1.0;
  mk.lifetime = rclcpp::Duration::from_seconds(0);
  for (const auto & [horiz, height] : traj_2d) {
    geometry_msgs::msg::Point p;
    p.x = mx + horiz * dir_x;
    p.y = my + horiz * dir_y;
    p.z = mz + height;
    mk.points.push_back(p);
  }
  traj_pub_->publish(mk);
}

bool ManagerNode::trajectoryHitsWindmill(double mx, double my, double mz) const
{
  const double dx = target_x_ - mx;
  const double dy = target_y_ - my;
  const double norm_xy = std::hypot(dx, dy);
  const double dir_x = (norm_xy > 0.01) ? dx / norm_xy : 1.0;
  const double dir_y = (norm_xy > 0.01) ? dy / norm_xy : 0.0;
  projectile_motion::GafProjectileSolver solver(bullet_speed_, friction_);
  const auto traj_2d = solver.computeTrajectory(target_pitch_up_, last_horizontal_, 48);
  for (const auto & [horiz, height] : traj_2d) {
    if (pointInVerticalCylinder(
        mx + horiz * dir_x, my + horiz * dir_y, mz + height,
        windmill_x_, windmill_y_, windmill_radius_,
        windmill_z_min_, windmill_z_max_))
    {
      return true;
    }
  }
  return false;
}

void ManagerNode::clearVisualization()
{
  // 抄自 ITL lob_shot_manager_node.cpp clearVisualization
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker del;
  del.header.frame_id = map_frame_;
  del.header.stamp = now();
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  del.ns = "lob_shot";
  markers.markers.push_back(del);
  viz_pub_->publish(markers);
}

void ManagerNode::publishVisualization()
{
  // 抄自 ITL lob_shot_manager_node.cpp publishVisualization，帧名改为 base_link。
  if (state_ == State::IDLE || state_ == State::WAITING_TF || state_ == State::INHIBITED) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  auto stamp = now();

  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = stamp;
    m.ns = "lob_shot";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = target_x_;
    m.pose.position.y = target_y_;
    m.pose.position.z = target_z_;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.15;
    m.color.r = 1.0;
    m.color.g = 0.2;
    m.color.b = 0.2;
    m.color.a = 1.0;
    markers.markers.push_back(m);
  }

  geometry_msgs::msg::TransformStamped tf_map_base;
  try {
    tf_map_base = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    viz_pub_->publish(markers);
    return;
  }

  const double gimbal_x = tf_map_base.transform.translation.x;
  const double gimbal_y = tf_map_base.transform.translation.y;
  const double gimbal_z = tf_map_base.transform.translation.z;

  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = stamp;
    m.ns = "lob_shot";
    m.id = 1;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.02;
    m.color.r = 0.2;
    m.color.g = 1.0;
    m.color.b = 0.2;
    m.color.a = 0.8;
    geometry_msgs::msg::Point p1;
    p1.x = gimbal_x;
    p1.y = gimbal_y;
    p1.z = gimbal_z + 0.3;
    geometry_msgs::msg::Point p2;
    p2.x = target_x_;
    p2.y = target_y_;
    p2.z = gimbal_z + 0.3;
    m.points.push_back(p1);
    m.points.push_back(p2);
    markers.markers.push_back(m);
  }

  bool have_trajectory = (state_ == State::PITCH_AIMING ||
    state_ == State::PITCH_CONVERGING ||
    state_ == State::READY);
  if (have_trajectory && last_horizontal_ > 0.1) {
    projectile_motion::GafProjectileSolver solver(bullet_speed_, friction_);
    auto traj_2d = solver.computeTrajectory(target_pitch_up_, last_horizontal_, 60);
    geometry_msgs::msg::TransformStamped tf_map_muzzle;
    try {
      tf_map_muzzle = tf_buffer_->lookupTransform(map_frame_, muzzle_frame_, tf2::TimePointZero);
      const double mx = tf_map_muzzle.transform.translation.x;
      const double my = tf_map_muzzle.transform.translation.y;
      const double mz = tf_map_muzzle.transform.translation.z;
      const double dx = target_x_ - mx;
      const double dy = target_y_ - my;
      const double norm_xy = std::hypot(dx, dy);
      const double dir_x = (norm_xy > 0.01) ? dx / norm_xy : 1.0;
      const double dir_y = (norm_xy > 0.01) ? dy / norm_xy : 0.0;

      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = stamp;
      m.ns = "lob_shot";
      m.id = 3;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.03;
      if (state_ == State::READY) {
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 1.0;
        m.color.a = 1.0;
      } else {
        m.color.r = 1.0;
        m.color.g = 0.6;
        m.color.b = 0.0;
        m.color.a = 0.9;
      }
      for (auto & [horiz, height] : traj_2d) {
        geometry_msgs::msg::Point p;
        p.x = mx + horiz * dir_x;
        p.y = my + horiz * dir_y;
        p.z = mz + height;
        m.points.push_back(p);
      }
      markers.markers.push_back(m);

      if (!traj_2d.empty()) {
        auto & [last_x, last_h] = traj_2d.back();
        visualization_msgs::msg::Marker mp;
        mp.header.frame_id = map_frame_;
        mp.header.stamp = stamp;
        mp.ns = "lob_shot";
        mp.id = 4;
        mp.type = visualization_msgs::msg::Marker::SPHERE;
        mp.action = visualization_msgs::msg::Marker::ADD;
        mp.pose.position.x = mx + last_x * dir_x;
        mp.pose.position.y = my + last_x * dir_y;
        mp.pose.position.z = mz + last_h;
        mp.pose.orientation.w = 1.0;
        mp.scale.x = mp.scale.y = mp.scale.z = 0.1;
        if (state_ == State::READY) {
          mp.color.r = 0.0;
          mp.color.g = 1.0;
          mp.color.b = 0.0;
          mp.color.a = 1.0;
        } else {
          mp.color.r = 1.0;
          mp.color.g = 0.6;
          mp.color.b = 0.0;
          mp.color.a = 1.0;
        }
        markers.markers.push_back(mp);
      }
    } catch (const tf2::TransformException &) {
    }
  }

  viz_pub_->publish(markers);
}

void ManagerNode::publishFieldMarkers()
{
  // 三块装甲示意：上/下灰、中间高亮。瞄准球仍走 ITL publishVisualization 的 ns=lob_shot id=0。
  // 板面朝 −X（红方）。外形抄 2026 制作规范 V1.4.0 图 3-16/3-17（小 135×125、大 230×127）。
  // 倾角抄 2026 V2.2.0 图 4-11 侧视：相对竖直线后仰 27.5°（绕 +Y，板顶偏向 +X）。
  // 吊射平面/站位不画 Marker（用户不要标区）。
  visualization_msgs::msg::MarkerArray markers;
  const auto stamp = now();
  for (const char * ns : {"lob_zone", "shooter_pad"}) {
    visualization_msgs::msg::Marker del;
    del.header.frame_id = map_frame_;
    del.header.stamp = stamp;
    del.ns = ns;
    del.id = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(del);
  }
  const double tilt = 27.5 * M_PI / 180.0;
  const double qy = std::sin(tilt * 0.5);
  const double qw = std::cos(tilt * 0.5);
  struct Plate
  {
    int id;
    double z;
    double w;
    double h;
    float r;
    float g;
    float b;
    float a;
    const char * label;
  };
  const Plate plates[] = {
    {10, armor_top_z_, 0.135, 0.125, 0.55f, 0.55f, 0.62f, 0.50f, "上"},
    {11, target_z_, 0.230, 0.127, 1.00f, 0.82f, 0.12f, 0.95f, "中"},
    {12, armor_bot_z_, 0.230, 0.127, 0.55f, 0.55f, 0.62f, 0.50f, "下"},
  };
  for (const auto & p : plates) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = stamp;
    m.ns = "base_armor";
    m.id = p.id;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = target_x_;
    m.pose.position.y = target_y_;
    m.pose.position.z = p.z;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = qy;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = qw;
    m.scale.x = 0.04;
    m.scale.y = p.w;
    m.scale.z = p.h;
    m.color.r = p.r;
    m.color.g = p.g;
    m.color.b = p.b;
    m.color.a = p.a;
    m.lifetime = rclcpp::Duration::from_seconds(0);
    markers.markers.push_back(m);

    visualization_msgs::msg::Marker t;
    t.header = m.header;
    t.ns = "base_armor";
    t.id = p.id + 10;
    t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    t.action = visualization_msgs::msg::Marker::ADD;
    t.pose.position.x = target_x_ - 0.18;
    t.pose.position.y = target_y_;
    t.pose.position.z = p.z;
    t.pose.orientation.w = 1.0;
    t.scale.z = (p.id == 11) ? 0.16 : 0.11;
    t.color.r = p.r;
    t.color.g = p.g;
    t.color.b = p.b;
    t.color.a = 1.0;
    t.text = p.label;
    t.lifetime = rclcpp::Duration::from_seconds(0);
    markers.markers.push_back(t);
  }

  viz_pub_->publish(markers);
}

void ManagerNode::onTrigger(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->data) {
    triggered_ = true;
    cancel_ = false;
  } else {
    cancel_ = true;
  }
}

void ManagerNode::onFire(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fire_ = msg->data;
}

void ManagerNode::onUnlocked(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  unlocked_ = msg->data;
}

void ManagerNode::onDocked(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  docked_ = msg->data;
}

void ManagerNode::onAssembling(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  assembling_ = msg->data;
}

void ManagerNode::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  bool got_pitch = false;
  bool got_yaw = false;
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (msg->name[i] == pitch_joint_name_) {
      current_pitch_ = msg->position[i];
      got_pitch = true;
    } else if (msg->name[i] == yaw_joint_name_) {
      current_yaw_ = msg->position[i];
      got_yaw = true;
    }
  }
  joint_ok_ = got_pitch && got_yaw;
}

}  // namespace heavy_lob_shot
