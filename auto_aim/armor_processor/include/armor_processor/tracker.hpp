// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

#include <Eigen/Eigen>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>
#include <cfloat>

// [IMM-Phase A] 用 IMMTracker 替换原来的 ExtendedKalmanFilter
#include "armor_processor/imm_tracker.hpp"
#include "jlcv_interfaces/msg/armors.hpp"
#include "jlcv_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
/**
 * @class Tracker
 * @brief 跟踪器类
 *
 * - 负责选择初始装甲板、初始化 IMM、根据检测结果维护跟踪状态机
 * - 提供对接收装甲板消息进行匹配、更新和跳变处理的接口
 * - [IMM-Phase A] 内部滤波器由单 EKF 升级为 IMMTracker（三模型 IMM-EKF）
 */
class Tracker
{
    enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3, BASE_1 = 1};

public:
    Tracker(double max_match_distance, double max_match_yaw_diff);

    using Armors = jlcv_interfaces::msg::Armors;
    using Armor  = jlcv_interfaces::msg::Armor;

    void init(const Armors::SharedPtr & armors_msg);
    void update(const Armors::SharedPtr & armors_msg);

    int getK() const;

    int getLostThres() const { return lost_thre_; }

    // [IMM-Phase A] 核心滤波器：由单 EKF 替换为三模型 IMM-EKF
    // 原 ExtendedKalmanFilter ekf; 已移除
    std::unique_ptr<IMMTracker> imm;
    std::unique_ptr<IMMTracker> imm0;
    
    int    tracking_thres;  // 进入 TRACKING 状态所需的连续检测帧数
    double lost_thres;      // 进入 LOST 状态所需的持续丢失时间（秒）

    enum State {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } tracker_state;

    Armor       tracked_armor;
    std::string tracked_id;
    ArmorsNum   tracked_armors_num;

    double info_position_diff;
    double info_yaw_diff;

    Eigen::VectorXd measurement;

    // 状态向量：[xc, vxc, yc, vyc, za, vza, yaw, vyaw, r, axc, ayc, dz1, dz2]
    Eigen::VectorXd target_state;
    int task_mode;

    // 前哨战三层高度存储（用于可视化和控制）
    double first_z, second_z, third_z;
    // 另一对装甲板的半径估计（滑动平均）
    double another_r;
    // 前哨站板号匹配结果（0/1/2），非前哨站时恒为0
    int matched_id = 0;
    // 前哨站是否已完成三板重建
    bool outpost_rebuilt = false;
    bool robot_rebuilt = false;
    bool rebuilt;

    double yaw_diff0;
    double yaw_pre;

private:
    void initIMM(const Armor & a);          // [IMM-Phase A] 原 initEKF 改名
    void updateArmorsNum(const Armor & a);
    void handleArmorJump(const Armor & a);
    double orientationToYaw(const geometry_msgs::msg::Quaternion & q);
    Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

    /// 装甲板匹配结果（数据关联阶段产物）
    struct MatchResult
    {
        bool   found             = false;     ///< 是否存在同 ID 装甲板
        int    same_id_count     = 0;         ///< 同 ID 装甲板数量
        int    best_id           = 0;         ///< 最佳匹配的逻辑板号 i
        double min_position_diff = DBL_MAX;   ///< 最佳匹配的预测/观测位置差
        double min_yaw_diff      = DBL_MAX;   ///< 最佳匹配的预测/观测 yaw 差
        Armor  matched_armor;                 ///< 最佳匹配的装甲板
    };

    /**
     * @brief 基于 yaw 相位的装甲板匹配（OUTPOST_3 与 NORMAL_4 通用）
     *
     * 遍历所有同 ID 装甲板，对每块板枚举 k 个候选逻辑板号 i，
     * 选择与预测 yaw 相位差最小的组合。k 由 getK() 给出（前哨站 3、普通 4），
     * 因此前哨站与普通机器人共用同一套匹配逻辑。
     *
     * @param armors_msg  当前帧装甲板消息（要求已填好 armor.yaw）
     * @param prediction  IMM 预测得到的状态向量
     * @return MatchResult 匹配结果
     */
    MatchResult matchArmorByYaw(
        const Armors::SharedPtr & armors_msg, const Eigen::VectorXd & prediction);

    /// 根据当前匹配结果更新前哨站/机器人重建标志
    void updateRebuildState();
    /// 用匹配到的装甲板构造观测并执行 IMM 更新
    void applyMeasurement();
    /// 更新前哨站三板高度估计（基于 dz1/dz2）
    void updateOutpostHeights();
    /// 根据 yaw 匹配质量维护丢失阈值与 LOST 判定
    void updateLostThreshold();
    /// 按装甲板 ID 对半径 r 施加硬约束
    void applyRadiusClampByID();
    /// 跟踪状态机更新
    void updateStateMachine(bool matched);

    double max_match_distance_;
    double max_match_yaw_diff_;

    int detect_count_;
    int lost_count_;
    double last_yaw_;

    bool is_oupost_mode_ = false;
    bool outpost_id_seen_[3] = {false, false, false};
    bool robot_id_seen_[4] = {false, false, false, false};
    std::pair<double, double> outpost_z_seen_[3]  = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};   // 新增：每块板首次/最近观测到的 z


    // [IMM-Phase A] 初始 R/Q 矩阵保留（初始化时仍需要）
    Eigen::MatrixXd initial_R_;
    Eigen::MatrixXd initial_Q_;

    bool isOutpost(const jlcv_interfaces::msg::Armor & armor);
    // int matchOutpostArmorId(const Eigen::VectorXd & state, double observed_yaw, double observed_z);
    void rebuildOutpost(double z_now, int & id);
    bool checkOutpostRebuild() const;

    bool checkRobotRebuild() const;

    int lost_thre_;
    float id2dz(int id){
        if(id == 0) return 0.0f;
        else if(id == 1) return target_state(11);
        else if(id == 2) return target_state(12);
        else return 0.0f;
    };


};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
