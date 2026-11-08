// Copyright 2022 Chen Jun
#include "armor_processor/processor_node.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <angles/angles.h>

// STD
#include <memory>
#include <vector>
#include <fstream>
#include <iostream>
namespace rm_auto_aim
{
// `ArmorProcessorNode` 构造函数
// 该构造函数完成参数声明、跟踪器与 EKF 的初始设置、TF 订阅器与消息发布器的初始化。
ArmorProcessorNode::ArmorProcessorNode(const rclcpp::NodeOptions& options) : Node("armor_processor", options)
{



    // Maximum allowable armor distance in the XOY plane
    // XOY平面中允许的最大装甲板距离参数
    max_armor_distance_ = this->declare_parameter("max_armor_distance", 12.0);

    // Tracker
    // 初始化跟踪器参数
    
    double tau = this->declare_parameter("ekf.tau", 0.1);
    double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.5);  //0.15     //最大匹配距离
    double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 0.5);  //1.0      //最大匹配偏航差
    tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);      
    debug_ = this->declare_parameter("tracker.debug", true);                 
    tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);                    //跟踪阈值
    lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 2.0);  //0.3                  //丢失时间阈值  
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/detector/final_img", 10, std::bind(&ArmorProcessorNode::imageCallback, this, std::placeholders::_1));

    // EKF
    // xa = x_armor, xc = x_robot_center
    // state: xc, yc, zc, yaw, v_xc, v_yc, v_zc, v_yaw, r
    // measurement: xa, ya, za, yaw
    // f - Process function

    // EKF扩展卡尔曼滤波器初始化
    // xa = 装甲板x坐标, xc = 机器人中心x坐标
    // 状态向量: xc, yc, zc, yaw, v_xc, v_yc, v_zc, v_yaw, r
    // 观测向量: xa, ya, za, yaw

    
    // f - 状态转移函数
    auto f = [this, tau](const Eigen::VectorXd& x) {
        Eigen::VectorXd x_new = x;

        
        double decay = std::exp(-dt_ / tau); // 指数衰减因子，模拟加速度随时间的衰减

        x_new(0) += x(1) * dt_ + 0.5 * x(9) * dt_ * dt_;
        x_new(1) += x(9) * dt_ ;
        x_new(2) += x(3) * dt_ + 0.5 * x(10) * dt_ * dt_;
        x_new(3) += x(10) * dt_ ;
        x_new(4) += x(5) * dt_ ;
        x_new(6) += x(7) * dt_ ;

        x_new(9) *= decay;  // 加速度随时间衰减
        x_new(10) *= decay; // 加速度随时间衰减
        x_new(11) = x(11);
        x_new(12) = x(12);

        return x_new;
    };
    // J_f - Jacobian of process function 状态转移函数的雅可比矩阵
    auto j_f = [this,tau](const Eigen::VectorXd&) {

        Eigen::MatrixXd f(13, 13);
        double decay = std::exp(-dt_ / tau); // 指数衰减因子，模拟加速度随时间的衰减
        // clang-format off
        // 状态转移函数的雅可比矩阵，对应变速运动模型
        f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,   0.5 * dt_ * dt_,   0,                0,   0,// xc       0
              0,   1,   0,   0,   0,   0,   0,   0,   0,   dt_,               0,                0,   0,// v_xc     1
              0,   0,   1,   dt_, 0,   0,   0,   0,   0,   0,                 0.5 * dt_ * dt_,  0,   0,// yc       2
              0,   0,   0,   1,   0,   0,   0,   0,   0,   0,                 dt_,              0,   0,// v_yc     3
              0,   0,   0,   0,   1,   dt_, 0,   0,   0,   0,                 0,                0,   0,// zc       4
              0,   0,   0,   0,   0,   1,   0,   0,   0,   0,                 0,                0,   0,// v_zc     5
              0,   0,   0,   0,   0,   0,   1,   dt_, 0,   0,                 0,                0,   0,// yaw      6
              0,   0,   0,   0,   0,   0,   0,   1,   0,   0,                 0,                0,   0,// v_yaw    7 
              0,   0,   0,   0,   0,   0,   0,   0,   1,   0,                 0,                0,   0,// r        8    
              0,   0,   0,   0,   0,   0,   0,   0,   0,   decay,             0,                0,   0,// a_xc     9
              0,   0,   0,   0,   0,   0,   0,   0,   0,   0,                 decay,            0,   0,// a_yc     10       
              0,   0,   0,   0,   0,   0,   0,   0,   0,   0,                 0,                1,   0,// dz1      11
              0,   0,   0,   0,   0,   0,   0,   0,   0,   0,                 0,                0,   1;// dz2      12
        
        return f;
    };
    // h - Observation function 观测函数（从状态向量中提取装甲板的位置和偏航角）
    auto h = [this](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(5);
        double xc = x(0), yc = x(2), yaw0 = x(6), r = x(8), dz1 = x(11), dz2 = x(12);
        int k = tracker_->getK();
        int matched_id = tracker_->matched_id;
        double angle = yaw0 + matched_id * 2 * M_PI / k; // 考虑装甲板数量对应的偏航角调整
        
        z(0) = xc - r * cos(angle);      // xa = xc - r * cos(yaw)
        z(1) = yc - r * sin(angle);      // ya = yc - r * sin(yaw)
        if(matched_id == 0){
            z(2) = x(4);
        } else if(matched_id == 1){
            z(2) = x(4) + dz1; // zb = zc + dz1，考虑装甲板高度差（前哨战特例）
        } else if(matched_id == 2){
            z(2) = x(4) + dz2; // zc = zc + dz2，考虑装甲板高度差（前哨战特例）
        } else {
            z(2) = x(4); // 默认情况，直接使用 zc
        }
        z(3) = cos(k * angle);           // cos(k * yaw) 用于处理不同装甲板数量对应的周期性偏航变化
        z(4) = sin(k * angle);           // sin(k * yaw) 用于处理不同装甲板数量对应的周期性偏航变化
        return z;
    };

    // J_h - Jacobian of observation function 观测函数的雅可比矩阵
    auto j_h = [this](const Eigen::VectorXd& x) {
        Eigen::MatrixXd h(5, 13);
        double yaw0 = x(6), r = x(8);
        int k = tracker_->getK();
        int matched_id = tracker_->matched_id;
        double angle = yaw0 + matched_id * 2 * M_PI / k; // 考虑装甲板数量对应的偏航角调整
        double dz1_dh = static_cast<double>(matched_id == 1); // dz1 对 h 的偏导数，考虑装甲板高度差（前哨战特例）
        double dz2_dh = static_cast<double>(matched_id == 2); // dz2 对 h 的偏导数，考虑装甲板高度差（前哨战特例）

        // clang-format off
        // 观测函数的雅可比矩阵，对应装甲板位置和偏航角的观测模型
        //    xc   vxc   yc   vyc  zc  vzc  yaw           vyaw    r            a_xc   a_yc     dz1  dz2
        h <<  1,   0,   0,   0,   0,   0,   r*sin(angle),    0,   -cos(angle),   0,     0,     0,  0,// xa对状态的偏导
            0,   0,   1,   0,   0,   0,   -r*cos(angle),     0,   -sin(angle),   0,     0,     0,  0,// ya对状态的偏导
            0,   0,   0,   0,   1,   0,   0,                 0,    0,          0,     0,     dz1_dh, dz2_dh,// za对状态的偏导
            0,   0,   0,   0,   0,   0,   -k*sin(k * angle), 0,    0,          0,     0,     0,    0,// yaw对状态的偏导
            0,   0,   0,   0,   0,   0,   k*cos(k * angle),  0,    0,          0,     0,     0,    0;// yaw对状态的偏导
        // clang-format on
        return h;
    };
    double s2qdz = declare_parameter("ekf.sigma2_q_dz", 1e-2); // dz 的过程噪声方差
    // --- Model 0: SteadyMotion ---
    double s2qxyz_0 = declare_parameter("imm.model0.sigma2_q_xyz", 7e-3);
    double s2qyaw_0 = declare_parameter("imm.model0.sigma2_q_yaw", 4e-6);
    double s2qr_0   = declare_parameter("imm.model0.sigma2_q_r",   80.0);
    double s2qa_0   = declare_parameter("imm.model0.sigma2_q_a",   4e-6);
    // --- Model 1: TranslationalManeuver（沿用原有参数） ---
    s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 7e-3);
    s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 4e-6);
    s2qr_   = declare_parameter("ekf.sigma2_q_r",   80.0);
    s2qa_   = declare_parameter("ekf.sigma2_q_a",   4e-2);
    // --- Model 2: SpinManeuver ---
    double s2qxyz_2 = declare_parameter("imm.model2.sigma2_q_xyz", 7e-3);
    double s2qyaw_2 = declare_parameter("imm.model2.sigma2_q_yaw", 4e-2);  // yaw 噪声放大50倍
    double s2qr_2   = declare_parameter("imm.model2.sigma2_q_r",   80.0);
    double s2qa_2   = declare_parameter("imm.model2.sigma2_q_a",   4e-6);
    std::cout << "s2qdz: " << s2qdz << std::endl; 
    std::cout << "============" << "MODEL0" << "=============" << std::endl;
    std::cout << "s2qxyz_0: " << s2qxyz_0 << std::endl;
    std::cout << "s2qyaw_0: " << s2qyaw_0 << std::endl;
    std::cout << "s2qr_0: " << s2qr_0 << std::endl;
    std::cout << "s2qa_0: " << s2qa_0 << std::endl;

    std::cout << "============" << "MODEL1" << "=============" << std::endl;
    std::cout << "s2qxyz_1: " << s2qxyz_ << std::endl;
    std::cout << "s2qyaw_1: " << s2qyaw_ << std::endl;
    std::cout << "s2qr_1: " << s2qr_ << std::endl;
    std::cout << "s2qa_1: " << s2qa_ << std::endl;

    std::cout << "============" << "MODEL2" << "=============" << std::endl;
    std::cout << "s2qxyz_2: " << s2qxyz_2 << std::endl;
    std::cout << "s2qyaw_2: " << s2qyaw_2 << std::endl;
    std::cout << "s2qr_2: " << s2qr_2 << std::endl;
    std::cout << "s2qa_2: " << s2qa_2 << std::endl;
    std::cout << "tau: " << tau << std::endl;
    // --- 构造 Q 矩阵的通用 lambda（按参数生成11×11 Q）---
    // 注意：捕获 this 是为了访问 dt_；每次 u_q 被调用时 dt_ 已更新
    auto make_u_q = [this, s2qdz](double s2xyz, double s2yaw, double s2r, double s2a) {
        return [this, s2xyz, s2yaw, s2r, s2a, s2qdz]() -> Eigen::MatrixXd {
            Eigen::MatrixXd q(13, 13);
            double t = dt_, x = s2xyz, y = s2yaw, r = s2r, a = s2a;
            double q_x_x   = pow(t, 5) / 20 * a;
            double q_x_vx  = pow(t, 4) / 8  * a;
            double q_vx_vx = pow(t, 3) / 3  * a;
            double q_y_y   = pow(t, 3) / 3  * y;
            double q_y_vy  = pow(t, 2) / 2  * y;
            double q_vy_vy = t * y;
            double q_z_z   = pow(t, 3) / 3  * x;
            double q_z_vz  = pow(t, 2) / 2  * x;
            double q_vz_vz = t * x;
            double q_r     = pow(t, 4) / 4  * r;
            double q_x_ax  = pow(t, 3) / 6  * a;
            double q_vx_ax = pow(t, 2) / 2  * a;
            double q_ax_ax = t * a;
            double q_dz    = s2qdz;  // dz 的过程噪声方差
            // clang-format off
            q <<  q_x_x,  q_x_vx,  0,       0,       0,       0,       0,       0,      0,    q_x_ax,   0,       0,      0,
                  q_x_vx, q_vx_vx, 0,       0,       0,       0,       0,       0,      0,    q_vx_ax,  0,       0,      0,
                  0,       0,       q_x_x,   q_x_vx,  0,       0,       0,       0,      0,    0,       q_x_ax,  0,      0,
                  0,       0,       q_x_vx,  q_vx_vx, 0,       0,       0,       0,      0,    0,       q_vx_ax, 0,      0,
                  0,       0,       0,       0,       q_z_z,   q_z_vz,  0,       0,      0,    0,       0,       0,      0,
                  0,       0,       0,       0,       q_z_vz,  q_vz_vz, 0,       0,      0,    0,       0,       0,      0,
                  0,       0,       0,       0,       0,       0,       q_y_y,   q_y_vy, 0,    0,       0,       0,      0,
                  0,       0,       0,       0,       0,       0,       q_y_vy,  q_vy_vy,0,    0,       0,       0,      0,
                  0,       0,       0,       0,       0,       0,       0,       0,      q_r,  0,       0,       0,      0,
                  q_x_ax,  q_vx_ax, 0,       0,       0,       0,       0,       0,      0,    q_ax_ax, 0,       0,      0,
                  0,       0,       q_x_ax,  q_vx_ax, 0,       0,       0,       0,      0,    0,       q_ax_ax, 0,      0,
                  0,       0,       0,       0,       0,       0,       0,       0,      0,    0,       0,       q_dz,   0,
                  0,       0,       0,       0,       0,       0,       0,       0,      0,    0,       0,       0,      q_dz;
            // clang-format on
            return q;
        };
    };
    // 生成三套 u_q 函数
    auto u_q_0 = make_u_q(s2qxyz_0, s2qyaw_0, s2qr_0, s2qa_0);  // SteadyMotion
    auto u_q_1 = make_u_q(s2qxyz_,  s2qyaw_,  s2qr_,  s2qa_);   // TranslationalManeuver
    auto u_q_2 = make_u_q(s2qxyz_2, s2qyaw_2, s2qr_2, s2qa_2);  // SpinManeuver
    // --- u_r（三个模型共用，与原来完全相同）---
    r_xyz_factor    = declare_parameter("ekf.r_xyz_factor", 4e-4);
    r_yaw           = declare_parameter("ekf.r_yaw",        5e-2);
    double r_yaw_factor = declare_parameter("ekf.r_yaw_factor", 1e-3);
    auto u_r = [this, r_yaw_factor](const Eigen::VectorXd & z) {
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(5, 5);
        
        // 1. 在相机坐标系下定义测量噪声协方差矩阵
        // 用 z.head<3>().norm() 近似代表目标距离，距离越远方差越大
        double d = z.head<3>().norm();
        double r_min = 1e-3;
        
        // 自定义系数：你可以将其暴露为 ROS 参数
        double xy_noise = r_xyz_factor; 
        double z_noise  = r_xyz_factor * 50.0; // 深度方向噪声设定为 XY 平面的 50 倍！
        
        Eigen::Matrix3d R_cam_pos = Eigen::Matrix3d::Zero();
        R_cam_pos(0, 0) = std::max(xy_noise * d, r_min);   // 相机左右方向
        R_cam_pos(1, 1) = std::max(xy_noise * d, r_min);   // 相机上下方向
        R_cam_pos(2, 2) = std::max(z_noise * d, r_min);    // 相机深度方向 (极其不可信)
        // 2. 将噪声矩阵旋转到目标坐标系下 (R_world = Q * R_cam * Q^T)
        // current_R_c2w_ 会在每次回调时更新
        Eigen::Matrix3d R_world_pos = current_R_c2w_ * R_cam_pos * current_R_c2w_.transpose();
        // 3. 填充到最终的 5x5 R 矩阵中
        R.block<3, 3>(0, 0) = R_world_pos;
        R(3, 3) = r_yaw_factor;
        R(4, 4) = r_yaw_factor;
        return R;
    };
    // --- 初始协方差矩阵（与原来相同）---
    Eigen::DiagonalMatrix<double, 13> p0;
    p0.setIdentity();
    // --- [IMM-Phase A] 构造 IMMTracker，替换原来的 ExtendedKalmanFilter ---
    tracker_->imm = std::make_unique<IMMTracker>(f, h, j_f, j_h,
    std::array<IMMTracker::VoidMatFunc, IMMTracker::N>{u_q_0, u_q_1, u_q_2},
    u_r, p0);
    tracker_->imm0 = std::make_unique<IMMTracker>(f, h, j_f, j_h,
    std::array<IMMTracker::VoidMatFunc, IMMTracker::N>{u_q_0, u_q_1, u_q_2},
    u_r, p0);    
    // [IMM-Phase A] 自适应 Q 已停用，以下两行注释掉
    // double adaptive_q_alpha = declare_parameter("ekf.adaptive_q_alpha", 0.35);
    // int window_size = declare_parameter("ekf.adaptive_q_window_size", 10);
    // tracker_->ekf.enableAdaptiveQ(adaptive_q_alpha, window_size);  ← 已删除
    // --- [IMM-Phase A] 新增：三个模型权重的调试话题发布器 ---
    // 在 processor_node.hpp 中需同步添加三个 Publisher 成员
    imm_weight_pub_[0] = this->create_publisher<std_msgs::msg::Float64>("/imm/weight_steady",      10);
    imm_weight_pub_[1] = this->create_publisher<std_msgs::msg::Float64>("/imm/weight_translational",10);
    imm_weight_pub_[2] = this->create_publisher<std_msgs::msg::Float64>("/imm/weight_spin",        10);

    //重置跟踪器服务
    reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/tracker/reset",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
            tracker_->tracker_state = Tracker::LOST;      //将跟踪器状态设为丢失
            response->success = true;
            v_yaw_filtered_ = 0.0;
            first_frame_ = true;
            last_raw_yaw_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Tracker reset!");
            return;
        });

    // Subscriber with tf2 message_filter     使用tf2 message_filter的订阅器
    // tf2 relevant                           tf2相关
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // 在调用waitForTransform之前创建计时器接口，避免异常
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // subscriber and filter  订阅器和滤波器
    armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
    target_frame_ = this->declare_parameter("target_frame", "world");     //目标坐标系
    tf2_filter_ =std::make_shared<tf2_filter>(armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
                                    this->get_node_clock_interface(), std::chrono::duration<int>(1));

        
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    // 注册tf2_ros::MessageFilter的回调函数，当变换可用时调用
    tf2_filter_->registerCallback(&ArmorProcessorNode::armorsCallback, this);

    // Measurement publisher (for debug usage)     // 测量值发布器（用于调试）
    measure_pub_ = this->create_publisher<jlcv_interfaces::msg::Measurement>("/tracker/measurement", 10);

    // Orientation publisher
    orien_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/debug/orientation", 10);
    // Publisher      目标信息发布器
    target_pub_ = this->create_publisher<jlcv_interfaces::msg::Target>("/processor/target", rclcpp::SensorDataQoS());

    // -------- 调试话题初始化 --------
    ekf_debug_pub_ = this->create_publisher<jlcv_interfaces::msg::EkfDebug>(
        "/ekf_debug", rclcpp::SensorDataQoS());

    v_yaw_old_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/debug/v_yaw_naive", 10);   // 朴素差分法（baseline）

    v_yaw_new_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/debug/v_yaw_ekf", 10);     // EKF估计值

    RCLCPP_INFO(this->get_logger(), "EKF debug publishers initialized!");

    // Visualization Marker Publisher           可视化标记发布器
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    position_marker_.ns = "position";
    position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
    position_marker_.color.a = 1.0;
    position_marker_.color.g = 1.0;
    linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns = "linear_v";
    linear_v_marker_.scale.x = 0.03;
    linear_v_marker_.scale.y = 0.05;
    linear_v_marker_.color.a = 1.0;
    linear_v_marker_.color.r = 1.0;
    linear_v_marker_.color.g = 1.0;
    angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.ns = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 1.0;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;
    armors_marker_.ns = "armors";
    armors_marker_.type = visualization_msgs::msg::Marker::CUBE;
    armors_marker_.scale.x = 0.03;
    armors_marker_.scale.y = 0.15;
    armors_marker_.scale.z = 0.12;
    armors_marker_.color.a = 1.0;
    armors_marker_.color.b = 1.0;
    armors_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/processor/marker", 10);
    
    recorder_ = std::make_shared<tools::Recorder>(2);

    // 日志信息，提示节点启动
    RCLCPP_INFO(this->get_logger(), "Starting ProcessorNode!");
}



// 装甲板数据回调函数
void ArmorProcessorNode::armorsCallback(const jlcv_interfaces::msg::Armors::SharedPtr armors_msg)
{
    // Tranform armor position from image target_pub_frame to world coordinate    // 将装甲板位置从图像坐标系变换到世界坐标系
    // std::cout<<"1111\n";

    RCLCPP_DEBUG(get_logger(), "当前相机坐标系 frame_id: %s", armors_msg->header.frame_id.c_str());

    // --- 新增：获取本帧相机到目标坐标系的旋转矩阵，用于动态 R 矩阵投影 ---
    try {
        geometry_msgs::msg::TransformStamped transform = tf2_buffer_->lookupTransform(
            target_frame_, armors_msg->header.frame_id, armors_msg->header.stamp, 
            rclcpp::Duration::from_nanoseconds(10000000));
        
        Eigen::Quaterniond q(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z
        );
        current_R_c2w_ = q.toRotationMatrix();
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "Could not get transform for R_c2w: %s", ex.what());
        current_R_c2w_ = Eigen::Matrix3d::Identity(); // 降级处理
    }
    // -----------------------------------------------------------

    for (auto& armor : armors_msg->armors)
    {

        // if(armor.number  == "outpost"){
        //     if(abs(tracker_->first_z - armor.pose.position.y) > 0.08){

        //         tracker_->third_z = tracker_->second_z;
        //         tracker_->second_z = tracker_->first_z;
        //         tracker_->first_z = armor.pose.position.y;
        //     }
        //     // std::fstream f;
        //     // f.open("./data.txt", std::ios::out | std::ios::app);
        //     // f<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << armor.pose.position.z << " " << tracker_->first_z << " " << tracker_->second_z << " " << tracker_->third_z << std::endl; 
        //     // f.close();
        

        // }

        // 转换装甲板位置到世界坐标系
        geometry_msgs::msg::PoseStamped ps;
        ps.header = armors_msg->header;
        ps.pose = armor.pose;
        // RCLCPP_ERROR(get_logger(), "%s",ps.header);
        try
        {
        armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;      //执行坐标变换
        // std::cout<<"x:"<<armor.pose.position.x<<std::endl;
        // std::cout<<"y:"<<armor.pose.position.y<<std::endl;
        // std::cout<<"z:"<<armor.pose.position.z<<std::endl;
        }
        catch (const tf2::ExtrapolationException& ex)
        {
        RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
        return;
        }


    }

    // geometry_msgs::msg::TransformStamped transform = tf2_buffer_->lookupTransform(
    //     target_frame_, "pitch_link", armors_msg->header.stamp,rclcpp::Duration::from_nanoseconds(100000000));

    // recorder_->record(Eigen::Quaterniond(transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w),armors_msg->header.stamp); //记录装甲板数据时间戳

    // 异常装甲板过滤代码    
    // Filter abnormal armors
    // armors_msg->armors.erase(
    //     std::remove_if(armors_msg->armors.begin(), armors_msg->armors.end(),
    //                    [this](const jlcv_interfaces::msg::Armor& armor) {
    //                      return abs(armor.pose.position.z) > 1.5 ||
    //                             Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
    //                                 max_armor_distance_;
    //                    }),
    //     armors_msg->armors.end());

    // Init message     初始化消息
    jlcv_interfaces::msg::Measurement measure_msg;    //测量消息
    jlcv_interfaces::msg::Target target_msg;          //目标消息
    rclcpp::Time time = armors_msg->header.stamp;
    target_msg.header.stamp = time;                   //目标消息时间戳
    target_msg.header.frame_id = target_frame_;       //目标消息坐标系
    target_msg.camera_id = armors_msg->camera_id;     //相机ID

    // Update tracker      更新跟踪器
    if (tracker_->tracker_state == Tracker::LOST)
    {
        //如果跟踪器处于丢失状态，则初始化跟踪器
        tracker_->init(armors_msg);
        target_msg.tracking = false;    //未跟踪到目标
    }
    else
    {
        //计算时间间隔
        dt_ = (time - last_time_).seconds();
        if (dt_ <= 0 || dt_ > 0.1){
            dt_ = 0.01;
        }
        
        tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);    //计算丢失阈值
        // tracker_->lost_thres = 15;
        tracker_->update(armors_msg);                                       //更新跟踪器状态

        // Publish measurement      发布测量值（用于调试）
        measure_msg.x = tracker_->measurement(0);
        measure_msg.y = tracker_->measurement(1);
        measure_msg.z = tracker_->measurement(2);
        measure_msg.yaw = atan2(tracker_->measurement(4), tracker_->measurement(3)) / tracker_->getK();
        measure_pub_->publish(measure_msg);

        if(!armors_msg->armors.empty()){
            orien_pub_->publish(armors_msg->armors[0].pose); //发布装甲板姿态信息
        }

        // Fill target message      填充目标消息
        if (tracker_->tracker_state == Tracker::DETECTING)
        {
            target_msg.tracking = false;
        }
        else if (tracker_->tracker_state == Tracker::TRACKING || tracker_->tracker_state == Tracker::TEMP_LOST)
        {
            target_msg.tracking = true;
            // Fill target message
            const auto& state = tracker_->target_state;
            target_msg.id = tracker_->tracked_id;
            target_msg.task_mode = tracker_->task_mode;
            target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
            target_msg.position.x = state(0);                       // xc
            target_msg.velocity.x = state(1);                       // v_xc
            target_msg.position.y = state(2);                       // yc 
            target_msg.velocity.y = state(3);                       // v_yc            
            target_msg.position.z = state(4);                        // zc        
            target_msg.velocity.z = state(5);                       // v_zc
            target_msg.yaw = angles::normalize_angle(state(6));                              // yaw
            target_msg.v_yaw = state(7);                            // v_yaw
            target_msg.radius_1 = state(8);                         // r
            target_msg.a_xc = state(9);                             // a_xc
            target_msg.a_yc = state(10);                            // a_yc
            target_msg.dz1 = state(11);                             // dz1
            target_msg.dz2 = state(12);                             // dz2
            target_msg.match_id = tracker_->matched_id;                   // matched_id
            // std::cout<<"result.r:"<<state(8)<<std::endl; 任书渝注释
            target_msg.radius_2 = tracker_->another_r;              // another_r
            // target_msg.second_z = tracker_->second_z;
            // target_msg.third_z = tracker_->third_z;
            // std::fstream f;
            // f.open("./data.txt", std::ios::out | std::ios::app);
            // f<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << target_msg.position.z << " " << tracker_->first_z << " " << tracker_->second_z << " " << tracker_->third_z << std::endl; 
            // f.close();
            // std::cout << target_msg.position.z << " " << tracker_->first_z << " " << tracker_->second_z << " " << tracker_->third_z << std::endl; 

            target_msg.first_z = tracker_->first_z;
            target_msg.second_z = tracker_->second_z;
            target_msg.third_z = tracker_->third_z;

            // target_msg.a_xc = state(9);                             //a_xc
            
            // target_msg.a_yc = state(10);                            //a_yc
            // if(target_msg.id=="outpost")
            // {
            //   target_msg.yaw =tracker_->measurement(3);
            // }
        }
        // [IMM-Phase A] 发布三个模型的权重（用于 PlotJuggler 验证模型切换效果）
        auto weights = tracker_->imm->getModelWeights();
        for (int i = 0; i < 3; ++i) {
            std_msgs::msg::Float64 w_msg;
            w_msg.data = weights[i];
            imm_weight_pub_[i]->publish(w_msg);
        }

    }
    last_time_ = time;                                          //更新上次时间戳

    // ============================================================
    // EKF 调试话题发布（用于PlotJuggler验证）
    // ============================================================
    debug_ = true;
    if ((tracker_->tracker_state == Tracker::TRACKING ||
        tracker_->tracker_state == Tracker::TEMP_LOST) && debug_)
    {
        const auto& state = tracker_->target_state;
        int k = tracker_->getLostThres();

        // -------- 构建调试消息 --------
        jlcv_interfaces::msg::EkfDebug debug_msg;
        debug_msg.header.stamp = time;
        debug_msg.header.frame_id = target_frame_;

        // 1. 原始观测量
        
        // 注意：改动后 measurement(3)=cos, measurement(4)=sin
        // 为了调试清晰，直接从tracked_armor重新取yaw
        tf2::Quaternion tf_q;
        tf2::fromMsg(tracker_->tracked_armor.pose.orientation, tf_q);
        double roll_dbg, pitch_dbg, yaw_dbg;
        tf2::Matrix3x3(tf_q).getRPY(roll_dbg, pitch_dbg, yaw_dbg);
        debug_msg.raw_yaw = yaw_dbg;   // 原始yaw，会有突变，用于对比

        // 2. cos/sin观测值
        debug_msg.cos_obs = tracker_->measurement(3);   // cos(k*yaw)
        debug_msg.sin_obs = tracker_->measurement(4);   // sin(k*yaw)

        // 3. EKF状态输出
        debug_msg.ekf_yaw   = state(6);   // EKF估计yaw
        debug_msg.ekf_v_yaw = state(7);   // EKF估计角速度（核心！）

        // 4. 从cos/sin反解yaw（应该连续无突变）
        debug_msg.yaw_recovered = atan2(
            tracker_->measurement(4),
            tracker_->measurement(3)
        ) / static_cast<double>(k);

        // 5. 新息（残差）——反映EKF对观测的信任程度
        // h(x_pri) 需要从EKF预测状态计算
        auto x_pri_yaw = tracker_->yaw_pre;   // 已有的预测yaw
        debug_msg.residual_cos = tracker_->measurement(3) - cos(k * x_pri_yaw);
        debug_msg.residual_sin = tracker_->measurement(4) - sin(k * x_pri_yaw);

        // 6. 协方差对角元素（反映EKF不确定度）
        auto P = tracker_->imm->get_P();
        debug_msg.p_yaw   = P(6, 6);
        debug_msg.p_v_yaw = P(7, 7);

        // 7. 当前k值
        debug_msg.k_value = k;
        double armor_yaw;
        int match_id = tracker_->matched_id;
        armor_yaw = angles::normalize_angle(target_msg.yaw + 2 * M_PI * match_id / target_msg.armors_num);
        // armor_yaw = target_msg.yaw;
        debug_msg.yaw_recovered = armor_yaw;
        debug_msg.xa = target_msg.position.x - target_msg.radius_1 * cos(armor_yaw);
        debug_msg.ya = target_msg.position.y - target_msg.radius_1 * sin(armor_yaw);
        debug_msg.dz1 = state(11);
        debug_msg.dz2 = state(12);
        
        if(match_id == 0){
            debug_msg.za = target_msg.position.z;
        } else if(match_id == 1){
            debug_msg.za = target_msg.position.z + state(11);
        } else if(match_id == 2){
            debug_msg.za = target_msg.position.z + state(12);
        } else {
            debug_msg.za = target_msg.position.z;
        }
        
        debug_msg.match_id = match_id;

        ekf_debug_pub_->publish(debug_msg);

        // -------- 朴素差分法 v_yaw（作为baseline对比） --------
        std_msgs::msg::Float64 v_yaw_naive_msg;
        if (!first_frame_ && dt_ > 1e-6) {
            // 用 shortest_angular_distance 防止角度跳变影响差分
            double yaw_delta = angles::shortest_angular_distance(last_raw_yaw_, state(6));
            v_yaw_naive_msg.data = yaw_delta / dt_;
        } else {
            v_yaw_naive_msg.data = 0.0;
            first_frame_ = false;
        }
        last_raw_yaw_ = state(6);
        v_yaw_old_pub_->publish(v_yaw_naive_msg);

        // -------- EKF估计的v_yaw --------
        std_msgs::msg::Float64 v_yaw_ekf_msg;
        v_yaw_ekf_msg.data = state(7);
        v_yaw_new_pub_->publish(v_yaw_ekf_msg);

        // -------- measure_msg 修正（改动后yaw字段反解真实角度） --------
        measure_msg.yaw = debug_msg.yaw_recovered;   // 用反解yaw替换cos值，避免混淆
    }
    // ============================================================


    target_pub_->publish(target_msg);                           //发布目标消息                   

    publishMarkers(target_msg);                                 //发布标记消息（用于可视化）   
}

// 发布可视化标记消息
void ArmorProcessorNode::publishMarkers(const jlcv_interfaces::msg::Target& target_msg)
{
visualization_msgs::msg::MarkerArray marker_array;

// Set headers for all markers 设置所有标记的头信息
position_marker_.header = target_msg.header;
linear_v_marker_.header = target_msg.header;
angular_v_marker_.header = target_msg.header;
armors_marker_.header = target_msg.header;

// std::cout<<"tracking:"<<target_msg.tracking<<std::endl;
if (target_msg.tracking)//如果正在跟踪目标
{
    // Extract target state 提取目标状态
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double first_z = target_msg.first_z, second_z = target_msg.second_z, third_z = target_msg.third_z;
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;

    if(target_msg.armors_num == 3)
    position_marker_.pose.position.z = (first_z + second_z + third_z) / 3;
    else
    position_marker_.pose.position.z = (za + second_z) / 2;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armors_marker_.action = visualization_msgs::msg::Marker::ADD;
    armors_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;

    // Draw armors
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;

    for (size_t i = 0; i < a_n; i++)
    {
        double tmp_yaw = yaw + i * (2 * M_PI / a_n) * target_msg.v_yaw/abs(target_msg.v_yaw);
        // Only 4 armors has 2 radius and height
        if (a_n == 4)
        {
            r = is_current_pair ? r1 : r2;
            p_a.z = is_current_pair ? za : second_z;
            is_current_pair = !is_current_pair;
        }
        else if(a_n == 3){
        r = r1;
        if(i == 0) p_a.z = first_z;
        else if (i == 1) p_a.z = second_z;
        else p_a.z = third_z;
        // std::cout << "h " << i << " " << p_a.z << std::endl; 任书渝注释
        //std::cout << first_z << " " << second_z << " " << third_z << std::endl; 
        }
        else
        {
            r = r1;
            p_a.z = za;
        }
        p_a.x = xc - r * cos(tmp_yaw);
        p_a.y = yc - r * sin(tmp_yaw);

        armors_marker_.id = i;
        armors_marker_.pose.position = p_a;
        tf2::Quaternion q;
        q.setRPY(0, target_msg.id == "outpost" ? -0.277 : 0.277, tmp_yaw);
        armors_marker_.pose.orientation = tf2::toMsg(q);
        marker_array.markers.emplace_back(armors_marker_);
    }
}
else
{
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    armors_marker_.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.emplace_back(armors_marker_);
}

marker_array.markers.emplace_back(position_marker_);
marker_array.markers.emplace_back(linear_v_marker_);
marker_array.markers.emplace_back(angular_v_marker_);
marker_pub_->publish(marker_array);
}

void ArmorProcessorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
frame_counter_++;

try {
    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr;
    

    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    
    // 获取OpenCV Mat对象
    cv::Mat image = cv_ptr->image;

    std::string label1= "Yaw Diff: " + std::to_string(tracker_->yaw_diff0);
    cv::putText(image, label1, cv::Point(500, 500), 
                cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 255, 0), 3);

    double mea_yaw = atan2(tracker_->measurement(4), tracker_->measurement(3)) / tracker_->getK();
    std::string label2 = "yaw_mea: " + std::to_string(mea_yaw);
    cv::putText(image, label2, cv::Point(500, 700), 
                cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 255, 0), 3);
    
    std::string label3= "yaw_ekf: " + std::to_string(tracker_->yaw_pre);
    cv::putText(image, label3, cv::Point(500, 900), 
                cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 255, 0), 3);

    std::string label4 = "yaw_tar: " + std::to_string(tracker_->target_state(6));
    cv::putText(image, label4, cv::Point(500, 100), 
                cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 255, 0), 3);
    if(tracker_->tracked_armor.number == "outpost"){
        for(int i=0; i<4; i++)
        cv::circle(image, cv::Point(tracker_->tracked_armor.kpx[i],tracker_->tracked_armor.kpy[i]), 3, cv::Scalar(0,255,0), -1);
    }
    if(tracker_->yaw_diff0 > 1.0){
        saveImage(image);
    }
    

} catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
} catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "处理图像时发生异常: %s", e.what());
}
}

void ArmorProcessorNode::saveImage(const cv::Mat& image)
{
    // 生成带时间戳的文件名
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&time);
    
    std::stringstream filename;
    filename << "image_"
            << std::put_time(&tm, "%Y%m%d_%H%M%S")
            << "_" << std::setfill('0') << std::setw(4) << frame_counter_++
            << ".jpg";
    
    // 保存为JPG格式（可改为.png）
    cv::imwrite("imgs/" + filename.str(), image);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorProcessorNode)
