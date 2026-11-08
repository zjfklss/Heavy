    // Copyright 2022 Chen Jun

    /**
     * @file processor_node.hpp
     * @brief 处理节点声明
     *
     * 本文件定义了 `ArmorProcessorNode` 节点类，
     * 负责接收检测器发布的装甲板信息，执行跟踪逻辑，
     * 并发布目标与可视化标记信息。
     */

    #ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
    #define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

    // ROS
    #include <message_filters/subscriber.h>
    #include <tf2_ros/buffer.h>
    #include <tf2_ros/create_timer_ros.h>
    #include <tf2_ros/message_filter.h>
    #include <tf2_ros/transform_listener.h>

    #include <rclcpp/rclcpp.hpp>
    #include <std_srvs/srv/trigger.hpp>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
    #include <visualization_msgs/msg/marker_array.hpp>
    #include <geometry_msgs/msg/pose.hpp>
    #include <opencv2/core.hpp>
    #include <opencv2/imgproc.hpp>

    // STD
    #include <memory>
    #include <string>
    #include <vector>

    #include "armor_processor/tracker.hpp"
    #include "jlcv_interfaces/msg/armors.hpp"
    #include "jlcv_interfaces/msg/target.hpp"
    #include "jlcv_interfaces/msg/measurement.hpp"
    #include "jlcv_interfaces/msg/ekf_debug.hpp"
    #include <std_msgs/msg/float64.hpp>

    #include "tools/recorder.hpp"

    namespace rm_auto_aim
    {
    using tf2_filter = tf2_ros::MessageFilter<jlcv_interfaces::msg::Armors>;
    class ArmorProcessorNode : public rclcpp::Node
    {
    public:
    explicit ArmorProcessorNode(const rclcpp::NodeOptions & options);
    

    private:
        void armorsCallback(const jlcv_interfaces::msg::Armors::SharedPtr armors_ptr);

        void publishMarkers(const jlcv_interfaces::msg::Target & target_msg);

        // Maximum allowable armor distance in the XOY plane
        double max_armor_distance_;

        int frame_counter_;

        // The time when the last message was received
        rclcpp::Time last_time_;
        double dt_;

        // Armor tracker
        double s2qxyz_, s2qyaw_, s2qr_,s2qa_;
        double r_xyz_factor, r_yaw;
        double lost_time_thres_;
        std::unique_ptr<Tracker> tracker_;
        
        // Reset tracker service
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

        // Subscriber with tf2 message_filter
        std::string target_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        message_filters::Subscriber<jlcv_interfaces::msg::Armors> armors_sub_;
        std::shared_ptr<tf2_filter> tf2_filter_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

        // Measurement publisher
        rclcpp::Publisher<jlcv_interfaces::msg::Measurement>::SharedPtr measure_pub_;

        // Orientation publisher
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr orien_pub_;

        // Publisher
        rclcpp::Publisher<jlcv_interfaces::msg::Target>::SharedPtr target_pub_;

        // Visualization marker publisher
        visualization_msgs::msg::Marker position_marker_;
        visualization_msgs::msg::Marker linear_v_marker_;
        visualization_msgs::msg::Marker angular_v_marker_;
        visualization_msgs::msg::Marker armors_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

        rclcpp::Publisher<jlcv_interfaces::msg::EkfDebug>::SharedPtr ekf_debug_pub_;

        // 单独发布v_yaw对比（方便PlotJuggler直接对比）
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr v_yaw_old_pub_;   // 原方案v_yaw
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr v_yaw_new_pub_;   // 新方案v_yaw

        // -------- 低通滤波器状态（用于对比） --------
        double v_yaw_filtered_ = 0.0;
        double alpha_ = 0.2;  // 低通滤波系数，作为baseline对比

        // -------- 用于保存上一帧yaw，手动差分计算naive v_yaw --------
        double last_raw_yaw_ = 0.0;
        bool first_frame_ = true;

        double debug_;
        /**
         * @brief 图像消息回调函数
         * @param msg 图像消息的共享指针
         */
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        
        /**
         * @brief 保存接收到的图像
         * @param image OpenCV格式的图像
         */
        void saveImage(const cv::Mat& image);

        std::shared_ptr<tools::Recorder>  recorder_;

        // [IMM-Phase A] IMM 模型权重调试话题发布器
        // weight_steady[0], weight_translational[1], weight_spin[2]
        std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 3> imm_weight_pub_;

        // -------- 动态测量噪声协方差所需 --------
        Eigen::Matrix3d current_R_c2w_ = Eigen::Matrix3d::Identity();


    };

    }  // namespace rm_auto_aim

    #endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_