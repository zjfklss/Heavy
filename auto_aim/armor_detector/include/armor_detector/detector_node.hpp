// Copyright 2022 Chen Jun
// Licensed under the MIT License.

/**
 * @file detector_node.hpp
 * @brief 装甲检测器的 ROS2 组件节点声明。
 *
 * 本节点订阅相机图像，运行 `Detector` 管道以定位装甲，使用 `PnPSolver`
 * 估计 3D 姿态，并发布 `jlcv_interfaces::msg::Armors` 与 RViz 可视化标记。
 * 当开启 `debug` 参数时，还会发布调试图像等主题。
 */

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/opencv.hpp>

// STD
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "armor_detector/detector.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "jlcv_interfaces/msg/armors.hpp"
#include "jlcv_interfaces/msg/game_status.hpp"
#include "armor_detector/detector_traditional.hpp"
#include "tools/recorder.hpp"

namespace rm_auto_aim
{

/**
 * @brief ArmorDetectorNode 类，负责装甲检测的 ROS2 节点。
 *
 * 主要功能：
 * - 订阅相机图像并使用 `Detector` 进行装甲检测；
 * - 利用 `PnPSolver` 估计装甲的 3D 姿态；
 * - 发布检测结果和可视化标记。
 */
class ArmorDetectorNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造装甲板检测节点
   * @param options 节点选项
   */
  ArmorDetectorNode(const rclcpp::NodeOptions & options);

  /**
   * @brief 析构函数，释放资源
   */
  ~ArmorDetectorNode();

  /// @brief 录像对象（可选）
  cv::VideoWriter record;

private:
  // ======================== 核心流程 ========================

  /**
   * @brief 图像回调函数：接收图像、检测装甲、PnP 解算并发布结果
   */
  void imageCallback(sensor_msgs::msg::Image::UniquePtr img_msg);

  /**
   * @brief 运行装甲检测流水线
   * @return 检测到的装甲列表
   */
  std::vector<Armor> detectArmors(sensor_msgs::msg::Image::UniquePtr img_msg);

  std::vector<Armor> detectArmors_traditional(sensor_msgs::msg::Image::UniquePtr img_msg);

  /**
   * @brief 初始化 YOLO 装甲检测器
   */
  std::unique_ptr<Detector> initYoloDetector();

  std::unique_ptr<TraditionalDetector> initDetector(uint8_t outpost_status);

  /**
   * @brief 发布 RViz 可视化标记
   */
  void publishMarkers();

  // ======================== 订阅器 ========================

  /// @brief 图像订阅器
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  /// @brief 相机信息订阅器
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  /// @brief 比赛状态订阅器
  rclcpp::Subscription<jlcv_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;

  // ======================== 发布器 ========================

  /// @brief 检测到的装甲板信息发布器
  rclcpp::Publisher<jlcv_interfaces::msg::Armors>::SharedPtr armors_pub_;

  /// @brief 可视化标记发布器
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // ======================== 检测器与解算器 ========================

  /// @brief 装甲板检测器
  std::unique_ptr<Detector> detector_;
  std::unique_ptr<TraditionalDetector> traditional_detector_;

  /// @brief PnP 求解器
  std::unique_ptr<PnPSolver> pnp_solver_;

  // ======================== 状态变量 ========================

  /// @brief 相机相关信息
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

  /// @brief 相机中心
  cv::Point2f cam_center_;

  /// @brief 图像传输方式（"raw" 或 "compressed"）
  std::string transport_;

  /// @brief 相机 ID（0 或 1）
  bool camera_id_ = 1;

  /// @brief 当前摄像头状态
  int camera_status = -1;

  /// @brief 是否翻转图像
  bool flip_;

  // ======================== 可视化 ========================

  /// @brief 装甲板可视化标记模板
  visualization_msgs::msg::Marker armor_marker_;

  /// @brief 可视化标记数组
  visualization_msgs::msg::MarkerArray marker_array_;

  // ======================== Debug 模式 ========================

  /// @brief 是否启用调试模式（通过 ROS 参数 "debug" 控制）
  bool debug_;
  bool debug_traditional_;

  /// @brief 调试参数变化监听器
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;

  /// @brief 调试参数回调句柄
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;

  /// @brief 创建所有调试消息发布器
  void createDebugPublishers();

  /// @brief 销毁所有调试消息发布器
  void destroyDebugPublishers();

  /// @brief 灯条调试数据发布器
  rclcpp::Publisher<jlcv_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;

  /// @brief 装甲板调试数据发布器
  rclcpp::Publisher<jlcv_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;

  /// @brief 二值化图像发布器
  image_transport::Publisher binary_img_pub_;
  /// @brief 数字图像发布器
  image_transport::Publisher number_img_pub_;
  /// @brief 最终传统检测图像发布器
  image_transport::Publisher final_traditional_img_pub_;

  /// @brief 最终结果图像发布器
  image_transport::Publisher final_img_pub_;

  /// @brief ROI 图像发布器
  image_transport::Publisher roi_img_pub_;

  // ======================== 数据采集（离线调试用） ========================

  /**
   * @brief 将原图与检测结果保存为调试图片
   */
  void saveDebugImage(const cv::Mat & raw_img, const std::vector<Armor> & armors);

  /// @brief 上次保存的时间
  std::chrono::steady_clock::time_point last_save_time_;

  /// @brief 已保存图片计数
  int saved_count_;

  /// @brief 包路径缓存
  std::string pkg_path;

  /// @brief 默认保存路径
  std::string default_save_path;

  /// @brief 识别器类型
  int detector_type_;

  std::shared_ptr<tools::Recorder>  recorder_;
  rclcpp::Time t_{0, 0, RCL_ROS_TIME};
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
