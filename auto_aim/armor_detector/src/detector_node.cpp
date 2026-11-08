// Copyright 2022 Chen Jun
// Licensed under the MIT License.

/**
 * @file detector_node.cpp
 * @brief 实现装甲检测节点的 ROS2 组件。
 * @details 主要负责实现detector_node.hpp中声明的方法。
 * 
 * 本源文件实现了 `detector_node.hpp` 中声明的逻辑：
 * - 参数初始化、订阅相机图像和相机信息；
 * - 初始化 `Detector` 和 `PnPSolver`；
 * - 发布检测到的装甲（`jlcv_interfaces::msg::Armors`）以及可视化标记和调试图像主题。
 */
// ROS2
#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rcpputils/filesystem_helper.hpp"

// Eigen
#include <Eigen/Eigen>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <ctime>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"
#include "armor_detector/detector_traditional.hpp"

using namespace std::chrono_literals;

namespace rm_auto_aim
{

// 把相机发来的 Image 统一解码成 3 通道 RGB 的 cv::Mat。
// 相机发布 bayer_rg8（单帧只有 rgb8 的 1/3 带宽，实测能跑满 165fps），
// 拜耳插值放在这里做一次；rgb8 / bgr8 / mono8 也一并兼容。
static cv::Mat toRgbMat(const std::shared_ptr<const sensor_msgs::msg::Image> & msg)
{
	const std::string & enc = msg->encoding;

	if (enc == "bayer_rg8" || enc == "bayer_bg8" || enc == "bayer_gr8" || enc == "bayer_gb8")
	{
		cv::Mat bayer(msg->height, msg->width, CV_8UC1,
		              const_cast<unsigned char *>(msg->data.data()), msg->step);
		// 厂商 BayerRG8(RGGB) 与 OpenCV 命名相反，R/B 要对调
		int code = cv::COLOR_BayerBG2RGB;  // bayer_rg8(RGGB)
		if (enc == "bayer_bg8")
			code = cv::COLOR_BayerRG2RGB;
		else if (enc == "bayer_gr8")
			code = cv::COLOR_BayerGB2RGB;
		else if (enc == "bayer_gb8")
			code = cv::COLOR_BayerGR2RGB;
		cv::Mat rgb;
		cv::cvtColor(bayer, rgb, code);
		return rgb;
	}

	if (enc == "mono8")
	{
		auto gray = cv_bridge::toCvShare(msg, "mono8")->image;
		cv::Mat rgb;
		cv::cvtColor(gray, rgb, cv::COLOR_GRAY2RGB);
		return rgb;
	}

	if (enc == "bgr8")
	{
		auto bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::Mat rgb;
		cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
		return rgb;
	}

	// rgb8 及其它格式交给 cv_bridge（rgb8 时是零拷贝）
	return cv_bridge::toCvShare(msg, "rgb8")->image;
}

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options) : Node("armor_detector", options)
{
	RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

	// Detector
	detector_ = initYoloDetector();

	traditional_detector_ = initDetector(0);

	// Subscriptions transport type 
	// 决定图像传输方式，压缩还是原图
	transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

	// Visualization Marker Publisher
	// Rviz相关话题信息发布
	// See http://wiki.ros.org/rviz/DisplayTypes/Marker
	// 装甲识别信息可视化
	armor_marker_.ns = "armors";
	armor_marker_.action = visualization_msgs::msg::Marker::ADD;
	armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
	armor_marker_.scale.x = 0.03;
	armor_marker_.scale.y = 0.15;
	armor_marker_.scale.z = 0.12;
	armor_marker_.color.a = 1.0;
	armor_marker_.color.r = 1.0;
	armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

	marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);


	// Debug Publishers
	// 默认启动调试信息发布
	debug_ = this->declare_parameter("debug", true);
	detector_type_ = this->declare_parameter("detector_type", 1); // 0: "yolo" or 1: "traditional"
	debug_traditional_ = this->declare_parameter("debug_traditional", true);
	flip_ = this->declare_parameter("flip", false);
	

	if (debug_ || debug_traditional_)
	{
		RCLCPP_INFO(this->get_logger(), "debug publishers created");
		createDebugPublishers();
	}


	// Debug param change moniter
	debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
	debug_cb_handle_ = debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter& p) {
		debug_ = p.as_bool();
		debug_ ? createDebugPublishers() : destroyDebugPublishers(); //debug 为 true 创建发布者，否则销毁
	});
	cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		"/camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
		cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
		cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
		pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
		cam_info_sub_.reset();
		});  
	game_status_sub_ = this->create_subscription<jlcv_interfaces::msg::GameStatus>(//收节点信息，获取GameStatus
	"/game_status", rclcpp::SensorDataQoS(), 
	[this](jlcv_interfaces::msg::GameStatus::ConstSharedPtr game_status) {
		if(camera_status != game_status->camera_status || pnp_solver_ == nullptr)
		{
		// std::cout<<"camera_status:"<<camera_status<<"\n";
		camera_status = game_status->camera_status;
		camera_id_ = camera_status;
		if(pnp_solver_ != nullptr){
		
		cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
			"/camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
			cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
			cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
			pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
			
			});  
		std::cout << "I'm solving the angle." << std::endl;
		}
		// std::cout << "final_status:" << camera_status << "\n";
		}

		// save_dataset_ = this->declare_parameter("debug.save_dataset", false); // 默认关闭，防止赛场上误开
		// save_path_ = this->declare_parameter("debug.save_path", default_save_path);
		// save_interval_sec_ = this->declare_parameter("debug.save_interval", 4); // 默认4秒存一张

		// 构造函数中
		last_save_time_ = std::chrono::steady_clock::now();
		saved_count_ = 0;

		// 如果开启保存且路径不存在，建议手动创建一下，或者使用 std::filesystem (需要C++17)
		// if (save_dataset_) {
		//     RCLCPP_INFO(this->get_logger(), "Data saving enabled. Path: %s", save_path_.c_str());
		//     // 简单创建目录命令 (Linux only)
		//     // 新代码：
		//   try {
		//       if (!std::filesystem::exists(save_path_)) {
		//           std::filesystem::create_directories(save_path_);
		//           RCLCPP_INFO(this->get_logger(), "Created directory: %s", save_path_.c_str());
		//       }
		//   } catch (const std::filesystem::filesystem_error& e) {
		//       RCLCPP_ERROR(this->get_logger(), "Could not create directory: %s", e.what());
		//   }
		// }
		detector_->enemy_color = game_status->enemy_color;

	});  

	// [优化 1] 定义极致低延迟的 QoS
	// History: KeepLast(1) -> 只保留最新一帧，杜绝积压
	// Reliability: BestEffort -> 允许丢包，保证实时性
	// Durability: Volatile -> 不持久化
	rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
	qos_profile.best_effort();
	qos_profile.durability_volatile();

	// [优化 2] 使用原生 Subscription 替代 image_transport 以支持 Intra-process Zero-Copy
	// 注意：需要发射端(相机驱动)也是 Component 且开启了 use_intra_process_comms
	img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
		"/image_raw", 
		qos_profile, 
		std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

	// Armors Publisher
	armors_pub_ = this->create_publisher<jlcv_interfaces::msg::Armors>(
		"/detector/armors", 
		qos_profile);

	
    pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");

    default_save_path = pkg_path + "/data/debug_images/";

	recorder_ = std::make_shared<tools::Recorder>(1);

}
ArmorDetectorNode::~ArmorDetectorNode()
{
	record.release();
}

void ArmorDetectorNode::imageCallback(sensor_msgs::msg::Image::UniquePtr img_msg)
{

	auto armors_msg = std::make_unique<jlcv_interfaces::msg::Armors>();
	
	armors_msg->header = armor_marker_.header = img_msg->header;

    std::vector<Armor> armors;

	if(detector_type_ == 0){
		// RCLCPP_INFO(this->get_logger(), "使用yolo识别器");
		armors = detectArmors(std::move(img_msg));
	}
	else if(detector_type_ == 1){
		// RCLCPP_INFO(this->get_logger(), "使用传统识别器");
		armors = detectArmors_traditional(std::move(img_msg));
	}else{
		// RCLCPP_ERROR(this->get_logger(), "未知的识别器类型: %d  默认使用yolo", detector_type_);
		armors = detectArmors(std::move(img_msg));		
	}


	// std::shared_ptr<sensor_msgs::msg::Image> shared_img = std::move(img_msg);
	// auto img = cv_bridge::toCvShare(shared_img, "rgb8")->image;

	/**
	 * 新增加的数据采集逻辑
	// RCLCPP_INFO(this->get_logger(), "当前数据采集状态为: %s", save_dataset_ ? "开启":"关闭");
	// if (save_dataset_) {
	//       auto now = std::chrono::steady_clock::now();
	//       auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_save_time_).count();
			
	//       std::cout << "上一次的保存时间为" << last_save_time_.time_since_epoch().count() << std::endl;
	//       std::cout << "现在的时间为" << now.time_since_epoch().count() << std::endl;
	//       // RCLCPP_INFO(this->get_logger(), "现在的保存间隔时间为 %ld 秒", duration);
		
	//       // 检查时间间隔 (2~6s，这里用 save_interval_sec_ 控制)
	//       if (duration > save_interval_sec_) {
	//         RCLCPP_INFO(this->get_logger(), "现在的保存间隔时间为 %ld 秒", duration);
	//           // 只有当检测到装甲板时才保存，还是不管有没有都保存？
	//           // 建议：只保存“检测到装甲板”的帧，因为你要检查的是角点准不准。
	//           if (!armors.empty()) {
	//               saveDebugImage(img, armors); // 调用下面的辅助函数
	//               last_save_time_ = now;
	//               RCLCPP_INFO(this->get_logger(), "保存成功！时间刷新！");
	//           }
	//       }
	//   }
	*/

	/// 使用PnP求解器计算每个检测到的装甲板的位姿并发布结果


	if (pnp_solver_ != nullptr)
	{
		// [关键修改] 使用 unique_ptr 创建消息，这是 ROS 2 零拷贝发布的必要条件

		armors_msg->camera_id = camera_id_;
		
		// 清理 marker (保持原有逻辑)
		marker_array_.markers.clear();
		armor_marker_.id = 0;

		for (const auto& armor : armors)
		{
			if(armor.color_id != detector_->enemy_color){
				continue;
			}
			jlcv_interfaces::msg::Armor armor_msg;
			for (auto & kp : armor.armor_keypoints)
			{
				armor_msg.kpx.push_back(kp.x);
				armor_msg.kpy.push_back(kp.y);
			}
			cv::Mat rvec, tvec;
		bool success = pnp_solver_->solvePnPPCABA(armor, rvec, tvec);
		// 历史解由 PnPSolver 内部按装甲板编号自管理（避免同帧多目标互相污染），
		// 此处不再写入全局 last_rvec/last_tvec。

			if (success)
			{
				armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
				armor_msg.number = armor.number;
				
				// 填充 Pose
				armor_msg.pose.position.x = tvec.at<double>(0);
				armor_msg.pose.position.y = tvec.at<double>(1); // 这里的 magic number -0.20 确认不需要优化吗？
				armor_msg.pose.position.z = tvec.at<double>(2);

				// 旋转矩阵计算
				cv::Mat rotation_matrix;
				cv::Rodrigues(rvec, rotation_matrix);
				tf2::Matrix3x3 tf2_rotation_matrix(
					rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
					rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
					rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
				tf2::Quaternion tf2_quaternion;
				tf2_rotation_matrix.getRotation(tf2_quaternion);
				
				armor_msg.pose.orientation.x = tf2_quaternion.x();
				armor_msg.pose.orientation.y = tf2_quaternion.y();
				armor_msg.pose.orientation.z = tf2_quaternion.z();
				armor_msg.pose.orientation.w = tf2_quaternion.w();
				
				armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);
				if (armor_msg.number == "outpost"){
					armor_msg.distance_to_image_center = 0;
				}
				// 填充 Marker
				armor_marker_.id++;
				armor_marker_.pose = armor_msg.pose;

				
				// armor_msg.reprojection_error = pnp_solver_->last_reprojection_error;
				// armor_msg.light_mean_length  = armor.light_mean_length;
				// armor_msg.refined            = armor.refined;


				armors_msg->armors.emplace_back(armor_msg);
				marker_array_.markers.emplace_back(armor_marker_);
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "PnP failed!");
			}
		}
		armors_msg->num = armors_msg->armors.size();



		armors_pub_->publish(std::move(armors_msg));

		publishMarkers();
	}
}

std::unique_ptr<Detector> ArmorDetectorNode::initYoloDetector()
{
	std::string pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
	std::string config_path = pkg_path + "/config/hero_yolo.yaml";
	std::cout << "Debug: 准备加载 YOLO 配置: " << config_path << std::endl;
	auto detector = std::make_unique<Detector>(config_path);
	std::cout << "Debug: YOLO 检测器初始化完成" << std::endl;
	return detector;
}


std::unique_ptr<TraditionalDetector> ArmorDetectorNode::initDetector(uint8_t outpost_status)
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int min_lightness = declare_parameter("binary_thres", 80, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;

  int enemy_color = declare_parameter("enemy_color", BLUE, param_desc);

  TraditionalDetector::LightParams l_params = { .min_ratio = declare_parameter("light.min_ratio", 0.08),
                                     .max_ratio = declare_parameter("light.max_ratio", 0.40),
                                     .max_angle = declare_parameter("light.max_angle", 40.0) };

  TraditionalDetector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.6),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.6),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.5),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.5),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)
  };


  auto traditional_detector = std::make_unique<TraditionalDetector>(min_lightness, enemy_color, l_params, a_params);

  // Init classifier
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/models/mlp_f1.onnx";
  auto label_path = pkg_path + "/models/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes;
  // if(!outpost_status){
  //   ignore_classes = this->declare_parameter("ignore_classes", std::vector<std::string>
  //                                                                     {"negative", "guard"});
  // }
  // else{
  //   ignore_classes = this->declare_parameter("ignore_classes", std::vector<std::string>
  //                                                                     {"negative"});
  // }
      ignore_classes = this->declare_parameter("ignore_classes", std::vector<std::string> {"negative"});
  
  traditional_detector->classifier = std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  std::cout << "Debug: 传统检测器初始化完成" << std::endl;

  return traditional_detector;
}


std::vector<Armor> ArmorDetectorNode::detectArmors(sensor_msgs::msg::Image::UniquePtr img_msg)
{
	auto header = img_msg->header;

	std::shared_ptr<sensor_msgs::msg::Image> shared_img = std::move(img_msg);
	cv::Mat img = toRgbMat(shared_img);
	// if(!img.empty())recorder_->record(img.clone(),img_msg->header.stamp);

	if (flip_)
	{
		cv::flip(img, img, -1);
	}

	auto armors = detector_->yolodetect(img);
	detector_->refineArmors(armors, img);
	
	/// 发布调试信息
	if (debug_)
	{
		auto final_time = this->now();
		auto latency = (final_time - header.stamp).seconds() * 1000;
		// record.write(img);

		cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);

		std::stringstream latency_ss;
		latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
		auto latency_s = latency_ss.str();
		cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

		detector_->draw_detections(img, armors);
		final_img_pub_.publish(cv_bridge::CvImage(header, "rgb8", img).toImageMsg());
	}

	return armors;
}


std::vector<Armor> ArmorDetectorNode::detectArmors_traditional(sensor_msgs::msg::Image::UniquePtr img_msg)
{

  auto header = img_msg->header;
  // 统一解码成 RGB（相机可能发的是 bayer_rg8），再转 BGR 给传统识别器
  std::shared_ptr<sensor_msgs::msg::Image> shared_img = std::move(img_msg);
  cv::Mat img = toRgbMat(shared_img);

  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

  if (flip_)
  {
    cv::flip(img, img, -1);
  }

  // if(!img.empty())recorder_->record(img.clone(),img_msg->header.stamp);
  // cv::imshow("img_",img);

  // Update params
  traditional_detector_->binary_thres = get_parameter("binary_thres").as_int();
  traditional_detector_->enemy_color = get_parameter("enemy_color").as_int();
  traditional_detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

  auto armors = traditional_detector_->detect(img);
  auto final_time = this->now();
  auto latency = (final_time - header.stamp).seconds() * 1000;//(final_time - header.stamp).seconds() * 1000;
//   RCLCPP_INFO(this->get_logger(), "Latency: %.2f ms", latency);

  // Publish debug info

  if (	debug_traditional_ )
  {
    // record.write(img);
    	binary_img_pub_.publish(cv_bridge::CvImage(header, "mono8", traditional_detector_->binary_img).toImageMsg());


    // Sort lights and armors data by x coordinate
    std::sort(traditional_detector_->debug_lights.data.begin(), traditional_detector_->debug_lights.data.end(),
              [](const auto& l1, const auto& l2) { return l1.center_x < l2.center_x; });
    std::sort(traditional_detector_->debug_armors.data.begin(), traditional_detector_->debug_armors.data.end(),
              [](const auto& a1, const auto& a2) { return a1.center_x < a2.center_x; });

    // lights_data_pub_->publish(traditional_detector_->debug_lights);
    // armors_data_pub_->publish(traditional_detector_->debug_armors);
    auto all_num_img = traditional_detector_->getAllNumbersImage();


    number_img_pub_.publish(cv_bridge::CvImage(header, "mono8", all_num_img).toImageMsg());
    traditional_detector_->drawResults(img);
    // Draw camera center
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);










	

    // Draw latency
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);


    final_traditional_img_pub_.publish(cv_bridge::CvImage(header, "bgr8", img).toImageMsg());


    // record_.write(img);
  }
  // cv::imshow("img",img);
  // cv::waitKey(1);
//   std::cout << "Debug: imageCallback finished" << std::endl;
  return armors;
}

void ArmorDetectorNode::createDebugPublishers()
{
	lights_data_pub_ = this->create_publisher<jlcv_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
	armors_data_pub_ = this->create_publisher<jlcv_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

	binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
	number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
	final_traditional_img_pub_ = image_transport::create_publisher(this, "/detector/final_traditional_img");

	final_img_pub_ = image_transport::create_publisher(this, "/detector/final_img");
	roi_img_pub_ = image_transport::create_publisher(this, "/detector/roi_img");

	RCLCPP_INFO(this->get_logger(), "debug publishers published");
}



void ArmorDetectorNode::destroyDebugPublishers()
{
	lights_data_pub_.reset();
	armors_data_pub_.reset();

	binary_img_pub_.shutdown();
	number_img_pub_.shutdown();
	final_traditional_img_pub_.shutdown();
	final_img_pub_.shutdown();
}


void ArmorDetectorNode::publishMarkers()
{
	marker_pub_->publish(marker_array_);
}

void ArmorDetectorNode::saveDebugImage(const cv::Mat & raw_img, const std::vector<Armor> & armors) {
	// 1. 拷贝一份图像用于画图，不影响原图处理
	cv::Mat debug_img = raw_img.clone();
	cv::Mat rgb_raw_img;
	cv::cvtColor(raw_img, rgb_raw_img, cv::COLOR_BGR2RGB);
	// 2. 遍历装甲板绘制角点
	for (const auto & armor : armors) {
		
		// 这里把所有点都画出来，并标上序号，方便检查顺序对不对
		for (size_t i = 0; i < armor.armor_keypoints.size(); ++i) {
			cv::Point2f pt = armor.armor_keypoints[i];
			
			// 画实心点
			cv::circle(debug_img, pt, 1, cv::Scalar(0, 255, 0), -1); 
			
			// 标序号 (0, 1, 2, 3) - 这对查bug非常重要！
			cv::putText(debug_img, std::to_string(i), pt + cv::Point2f(5, 5), 
						cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
		}
	}

	cv::Mat rgb_debug_img;
	cv::cvtColor(debug_img, rgb_debug_img, cv::COLOR_BGR2RGB);
	std::string pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
	std::string debug_save_path = pkg_path + "/data/debug_images/";
	std::string raw_save_path = pkg_path + "/data/raw_images/";
	// 3. 生成文件名 (时间戳 + 计数)
	// 格式: timestamp_count.jpg
	std::string debug_filename = debug_save_path+ std::to_string(this->now().nanoseconds()) + 
						"_" + std::to_string(saved_count_++) + "_debug" + ".jpg";

	std::string raw_filename = raw_save_path + std::to_string(this->now().nanoseconds()) + 
						"_" + std::to_string(saved_count_++) + "_raw" + ".jpg";
	
	// 4. 写入磁盘
	// 使用 imwrite。
	// 加上异常捕获防止IO错误崩掉节点
	try {
		cv::imwrite(debug_filename, rgb_debug_img);
		cv::imwrite(raw_filename, rgb_raw_img);
		RCLCPP_INFO(this->get_logger(), "Saved raw image: %s", raw_filename.c_str());
		RCLCPP_INFO(this->get_logger(), "Saved debug image: %s", debug_filename.c_str());
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Failed to save image: %s", e.what());
	}
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
