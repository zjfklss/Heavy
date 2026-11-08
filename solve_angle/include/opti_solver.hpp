#ifndef SOLVE_ANGLE__SOLVE_ANGLE_HPP_
#define SOLVE_ANGLE__SOLVE_ANGLE_HPP_

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <vector>

// 自定义消息
#include "jlcv_interfaces/msg/target.hpp"
#include "jlcv_interfaces/msg/game_status.hpp"

// 弹道解算
#include "ballistic_solver.hpp"

// MPC Planner（可选）
#include "solve_angle/planner.hpp"

namespace solve_angle {

class SolveAngle : public rclcpp::Node {
public:
    explicit SolveAngle(const rclcpp::NodeOptions& options);
    friend class SolveAngleTest;

private:
    // ==================== 参数 ====================

    struct Params {
        double debug_pitch_offset;
        double debug_yaw_offset;
        double max_yaw_error;       // 允许开火的最大偏航误差 (rad)
        double shoot_delay;         // 射击指令延迟 (s)
        double max_linear_error;    // 允许的最大横向线性误差 (m)
        double max_allow_angle;     // 近距离最大允许角度 (rad)
        double min_allow_angle;     // 远距离最小允许角度 (rad)
    } params_;

    // ==================== 内部类型 ====================

    struct HitPoint {
        double x, y, z;
        double yaw;  // 击打时刻装甲板朝向
    };

    struct Point3D {
        double x, y, z;
    };

    // ==================== 回调与发布 ====================

    void targetCallback(const jlcv_interfaces::msg::Target::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publishCommand(double pitch, double yaw, int shoot_flag);
    /// 目标丢失 / 解算失败时调用：最近一次有效指令新鲜（0.5s 内）则保持云台，
    /// 超时后停止发送；绝不发送绝对 (0,0)，shoot_flag 恒为 0
    void publishHoldCommand();

    // ==================== 状态机处理 ====================

    bool processTracking(const jlcv_interfaces::msg::Target::SharedPtr msg,
                         double& pitch_out, double& yaw_out);
    bool processOutpost(const jlcv_interfaces::msg::Target::SharedPtr msg,
                        double& pitch_out, double& yaw_out);
    bool processOutpostFallback(const jlcv_interfaces::msg::Target::SharedPtr msg,
                                double& pitch_out, double& yaw_out);
    bool processOutpostStatic(const jlcv_interfaces::msg::Target::SharedPtr msg,
                                double& pitch_out, double& yaw_out);
    bool processAssessment(const jlcv_interfaces::msg::Target::SharedPtr msg,
                           double& pitch_out, double& yaw_out);
    bool processAssessment2(const jlcv_interfaces::msg::Target::SharedPtr msg,
                            double& pitch_out, double& yaw_out);

    // ==================== 辅助函数 ====================

    /// 动态开火阈值：根据距离计算允许角度
    double getDynamicThreshold(double distance);

    /// Brent 二分法求自洽 predict_time
    /// @return 求解成功返回 predict_time，失败返回 nullopt
    std::optional<double> solvePredictTime(double cur_x, double cur_y,
                                           double cur_z, double vx, double vy);

    /// 计算考虑旋转后的预测击打点
    HitPoint calculateHitPoint(const jlcv_interfaces::msg::Target::SharedPtr msg,
                               double predict_time, int target_idx, double z_aim);

    // ==================== ROS2 接口 ====================

    rclcpp::Subscription<jlcv_interfaces::msg::Target>::SharedPtr    target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    joint_state_sub_;
    rclcpp::Subscription<jlcv_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr         gimbal_cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    aim_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr         gimbal_debug_pub_;

    // ==================== 模块 ====================

    std::unique_ptr<BallisticSolver> solver_;
    BallisticConfig solver_config_;

    // ==================== MPC Planner ====================

    bool use_mpc_ = false;
    std::unique_ptr<Planner> planner_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpc_debug_pub_;

    /// 生成 MPC 参考轨迹（基于目标匀速预测 + ballistic solver）
    void generateMPCReference(
        const jlcv_interfaces::msg::Target::SharedPtr msg,
        double pitch_offset, double yaw_offset,
        Trajectory& yaw_ref, Trajectory& pitch_ref);

    // ==================== 运行时状态 ====================

    rclcpp::Time last_shoot_time_;
    int    shoot_flag_    = 0;
    bool   suggest_fire_  = false;

    // 最近一次有效指令：目标丢失 / 解算失败时下发它保持云台，
    // 避免直接发 (0,0) 绝对角把云台拽回水平造成"莫名低头"
    bool         has_last_cmd_    = false;
    double       last_pitch_cmd_  = 0.0;
    double       last_yaw_cmd_    = 0.0;
    rclcpp::Time last_cmd_stamp_;
    double       cmd_hold_timeout_ = 0.5;  // 保持时长 (s)

    // 四板锁板防抖：上一帧锁定的物理板号 (0..3)，-1 表示未锁定
    int lock_plate_ = -1;

    // 掉头检测状态
    double last_vx_    = 0.0;
    double last_vy_    = 0.0;
    bool   is_turning_ = false;

    // 当前云台角度（由 jointStateCallback 更新）
    double curr_pitch = 0.0;
    double curr_yaw   = 0.0;

    // 相机 / 弹速（电控上报做 EMA，抑制 15.6~16.0 级抖动）
    int   camera_status_ = 0;
    float bullet_speed_  = 0.0f;
    bool  bullet_speed_inited_ = false;
    static constexpr float kBulletSpeedEmaAlpha = 0.15f;
    std::vector<double> three_z;
};

}  // namespace solve_angle

#endif  // SOLVE_ANGLE__SOLVE_ANGLE_HPP_
