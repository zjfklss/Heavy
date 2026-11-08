// Copyright 2022 Chen Jun

#include "armor_processor/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cfloat>
#include <memory>
#include <string>
#include <iostream>

namespace rm_auto_aim
{

Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST),
  tracked_id(std::string("")),
  measurement(Eigen::VectorXd::Zero(5)),
  target_state(Eigen::VectorXd::Zero(13)),
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(max_match_yaw_diff)
{

}

int Tracker::getK() const
{
    switch (tracked_armors_num) {
        case ArmorsNum::BALANCE_2:  return 2;
        case ArmorsNum::OUTPOST_3:  return 3;
        case ArmorsNum::NORMAL_4:   return 4;
        default:                    return 4;
    }
}

// ============================================================
// init：跟踪器初始化（选择初始目标，初始化 IMM）
// ============================================================
void Tracker::init(const Armors::SharedPtr & armors_msg)
{
    if (armors_msg->armors.empty()) return;

    // 选择距图像中心最近的装甲板作为初始目标
    double min_distance = DBL_MAX;
    tracked_armor = armors_msg->armors[0];

    for (auto & armor : armors_msg->armors) {
        armor.yaw = orientationToYaw(armor.pose.orientation);
    }
    for (const auto & armor : armors_msg->armors) {
        if (armor.distance_to_image_center < min_distance) {
            min_distance = armor.distance_to_image_center;
            tracked_armor = armor;
        }
    }

    // [IMM-Phase A] 初始化 IMM（原 initEKF）
    initIMM(tracked_armor);
    RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init IMM!");

    tracked_id = tracked_armor.number;
    tracker_state = DETECTING;
    updateArmorsNum(tracked_armor);
}

// ============================================================
// update：主更新函数
// [IMM-Phase A] 所有 ekf.predict/update/setState/set_P 调用
//               均替换为 imm.predict/update/setState/set_P
// ============================================================
void Tracker::update(const Armors::SharedPtr & armors_msg)
{
    // --- IMM 预测步骤（替换原 ekf.predict()）---
    Eigen::VectorXd imm_prediction = imm->predict();
    RCLCPP_DEBUG(rclcpp::get_logger("armor_processor"), "IMM predict");

    bool matched = false;
    target_state = imm_prediction;
    yaw_pre = imm_prediction(6);

    if (!armors_msg->armors.empty()) {
        // 预先解算所有装甲板的 yaw
        for (auto & armor : armors_msg->armors) {
            armor.yaw = orientationToYaw(armor.pose.orientation);
        }

        // NORMAL_4 进入即清除前哨站重建标志（原逻辑保留）
        if (tracked_armors_num == ArmorsNum::NORMAL_4) {
            outpost_rebuilt = false;
        }

        // --- 数据关联：基于 yaw 相位匹配（OUTPOST_3 / NORMAL_4 通用）---
        MatchResult m = matchArmorByYaw(armors_msg, imm_prediction);

        // 仅当观测位置仍接近旧板预测位置时才抑制板号跳变，避免侧面新板转入时被强行锁死在旧板号
        if (m.min_yaw_diff > max_match_yaw_diff_ && outpost_rebuilt &&
            m.min_position_diff < 0.15) {
            m.best_id = matched_id;
        }

        if (m.found) {
            tracked_armor = m.matched_armor;
        }
        yaw_diff0          = m.min_yaw_diff;
        info_position_diff = m.min_position_diff;
        info_yaw_diff      = m.min_yaw_diff;

        if (m.found) {
            matched    = true;
            matched_id = m.best_id;

            updateRebuildState();
            applyMeasurement();
            updateOutpostHeights();
            updateLostThreshold();
        } else {
            RCLCPP_DEBUG_THROTTLE(
                rclcpp::get_logger("armor_processor"), *rclcpp::Clock::make_shared(), 1000,
                "No Same ID (tracked=%s), unmatched frame", tracked_id.c_str());
        }
    }

    rebuilt = outpost_rebuilt || robot_rebuilt;

    applyRadiusClampByID();
    updateStateMachine(matched);
}

// ============================================================
// matchArmorByYaw：基于 yaw 相位的装甲板匹配（OUTPOST_3 / NORMAL_4 通用）
// 原 update() 中 OUTPOST_3 与 NORMAL_4 两个几乎相同的分支合并于此，
// 唯一差异（候选板号数量 3 / 4）由 getK() 统一处理。
// ============================================================
Tracker::MatchResult Tracker::matchArmorByYaw(
    const Armors::SharedPtr & armors_msg, const Eigen::VectorXd & prediction)
{
    MatchResult result;

    // 仅前哨站与普通机器人走相位匹配
    if (tracked_armors_num != ArmorsNum::OUTPOST_3 &&
        tracked_armors_num != ArmorsNum::NORMAL_4) {
        return result;
    }

    const double xc = prediction(0), yc = prediction(2);
    const double za = prediction(4), yaw0 = prediction(6);
    const double r = prediction(8);
    const int k = getK();
    auto predDz = [&](int id) -> double {
        if (id == 1 && prediction.size() > 11) return prediction(11);
        if (id == 2 && prediction.size() > 12) return prediction(12);
        return 0.0;
    };

    for (const auto & armor : armors_msg->armors) {
        if (armor.number != tracked_id) continue;
        result.same_id_count++;

        const auto & p = armor.pose.position;
        Eigen::Vector3d obs_pos(p.x, p.y, p.z);

        // 枚举 k 个候选逻辑板号，取 yaw 相位差最小者
        for (int i = 0; i < k; i++) {
            double angle_i  = yaw0 + i * (2.0 * M_PI / k);
            double y_diff_i = std::abs(angles::shortest_angular_distance(angle_i, armor.yaw));

            if (y_diff_i < result.min_yaw_diff) {
                result.min_yaw_diff = y_diff_i;

                Eigen::Vector3d pred_pos(
                    xc - r * cos(angle_i),
                    yc - r * sin(angle_i),
                    za + predDz(i));
                result.min_position_diff = (pred_pos - obs_pos).norm();
                result.matched_armor     = armor;
                result.best_id           = i;
            }
        }
    }

    result.found = result.same_id_count > 0;
    return result;
}

// ============================================================
// updateRebuildState：根据当前匹配结果更新前哨站/机器人重建标志
// ============================================================
void Tracker::updateRebuildState()
{
    if (tracked_armors_num == ArmorsNum::OUTPOST_3) {
        if (!outpost_id_seen_[matched_id]) {
            outpost_id_seen_[matched_id] = true;
            outpost_z_seen_[matched_id].first  = tracked_armor.pose.position.z;  // 记录该板的 z
            outpost_z_seen_[matched_id].second = tracked_armor.yaw;              // 记录该板的 yaw
        }
        if (!outpost_rebuilt && checkOutpostRebuild()) {
            // rebuildOutpost(tracked_armor.pose.position.z, matched_id);
            outpost_rebuilt = true;
        }
    } else if (tracked_armors_num == ArmorsNum::NORMAL_4) {
        robot_id_seen_[matched_id] = true;
        if (!robot_rebuilt && checkRobotRebuild()) {
            robot_rebuilt = true;
        }
    }
}

// ============================================================
// applyMeasurement：用匹配到的装甲板构造观测并执行 IMM 更新
// ============================================================
void Tracker::applyMeasurement()
{
    auto p = tracked_armor.pose.position;
    double measured_yaw = tracked_armor.yaw;
    int k = getK();
    measurement = Eigen::Matrix<double, 5, 1>(
        p.x, p.y, p.z,
        cos(k * measured_yaw), sin(k * measured_yaw));
    target_state = imm->update(measurement);
    another_r = 0.95 * another_r + 0.05 * target_state(8);
}

// ============================================================
// updateOutpostHeights：更新前哨站三板高度估计（基于 dz1/dz2）
// v_yaw 符号决定三块板的绕行顺序，从而决定 second/third 的 dz 偏移。
// ============================================================
void Tracker::updateOutpostHeights()
{
    if (tracked_armors_num != ArmorsNum::OUTPOST_3) return;

    int next_offset = (target_state(7) < 0) ? 1 : 2;
    int prev_offset = (target_state(7) < 0) ? 2 : 1;

    first_z  = target_state(4) + id2dz(matched_id);
    second_z = target_state(4) + id2dz((matched_id + next_offset) % 3);
    third_z  = target_state(4) + id2dz((matched_id + prev_offset) % 3);
}

// ============================================================
// updateLostThreshold：根据 yaw 匹配质量维护丢失阈值与 LOST 判定
// ============================================================
void Tracker::updateLostThreshold()
{
    if (info_yaw_diff > max_match_yaw_diff_ && rebuilt) {
        lost_thre_ += 1;
    } else if (info_yaw_diff < max_match_yaw_diff_) {
        if (lost_thre_ < 0) lost_thre_ = 0;
    }
    if (lost_thre_ > 200) {
        outpost_rebuilt = false;
        tracker_state   = LOST;
        lost_thre_      = 0;
    }
}

// ============================================================
// applyRadiusClampByID：按装甲板 ID 对半径 r 施加硬约束
// [Phase C 待评估] 这些硬裁剪在 RM 业务上是合理的安全边界，
// 但会破坏状态的高斯分布假设；Phase C 再决定是否改为软约束。
// ============================================================
void Tracker::applyRadiusClampByID()
{
    auto applyRadiusClamp = [this](double r_max_val, double r_min_val = 0.0) {
        bool changed = false;
        if (target_state(8) > r_max_val) {
            target_state(8) = r_max_val;
            changed = true;
        }
        if (r_min_val > 0.0 && target_state(8) < r_min_val) {
            target_state(8) = r_min_val;
            changed = true;
        }
        if (changed) {
            imm->setState(target_state);
            auto P = imm->get_P();
            P.row(8).setZero();
            P.col(8).setZero();
            P(8, 8) = 1e-4;
            imm->set_P(P);
        }
    };

    const auto & id = tracked_armor.number;
    if      (id == "1")                                          applyRadiusClamp(0.40, 0.15);
    else if (id == "2" || id == "3" || id == "4" || id == "5")  applyRadiusClamp(0.30, 0.12);
    else if (id == "guard")                                      applyRadiusClamp(0.35, 0.15);
    else if (id == "outpost")                                    applyRadiusClamp(0.29, 0.26);
    else                                                         applyRadiusClamp(0.40, 0.12);
}

// ============================================================
// updateStateMachine：跟踪状态机更新
// 修复：原先在 !matched && !rebuilt 时：
//   1) update() 先把状态强设为 TEMP_LOST
//   2) updateStateMachine 的 TEMP_LOST 分支因 rebuilt==false 走 else，
//      直接把 tracker_state 改回 TRACKING 并清零 lost_count_，导致锁错 ID 后永不 LOST。
// ============================================================
void Tracker::updateStateMachine(bool matched)
{
    if (tracker_state == DETECTING) {
        if (matched) {
            detect_count_++;
            if (detect_count_ > tracking_thres) {
                detect_count_ = 0;
                tracker_state = TRACKING;
            }
        } else {
            detect_count_ = 0;
            tracker_state = LOST;
        }
    } else if (tracker_state == TRACKING) {
        if (!matched) {
            tracker_state = TEMP_LOST;
            lost_count_++;
        }
    } else if (tracker_state == TEMP_LOST) {
        if (!matched) {
            lost_count_++;
            if (lost_count_ > lost_thres) {
                lost_count_   = 0;
                tracker_state = LOST;
            }
        } else {
            tracker_state = TRACKING;
            lost_count_   = 0;
        }
    }
}

// ============================================================
// initIMM：用初始装甲板信息初始化 IMM 所有子模型
// [IMM-Phase A] 原 initEKF 改名为 initIMM，内部调用 imm.setState
// ============================================================
void Tracker::initIMM(const Armor & a)
{
    double xa  = a.pose.position.x;
    double ya  = a.pose.position.y;
    double za  = a.pose.position.z;
    last_yaw_  = 0;
    double yaw = orientationToYaw(a.pose.orientation);

    

    target_state = Eigen::VectorXd::Zero(13);

    double r = 0.276;
    if (a.type   == "large")   r = 0.40;
    if (a.number == "outpost") r = 0.276;
    if (a.number == "sentry")  r = 0.35;

    double xc = xa + r * cos(yaw);
    double yc = ya + r * sin(yaw);
    another_r = r;

    // [思路 B] 初始化时把首次见到的板定为逻辑板 0：
    //   matched_id_ = 0, state(6) = observed_yaw
    double dz1 = 0.0, dz2 = 0.0;
    target_state << xc, 0, yc, 0, za, 0, yaw, 0, r, 0, 0, dz1, dz2;

    imm->reset();
    imm->setState(target_state);



    matched_id       = 0;
    outpost_rebuilt  = false;
    robot_rebuilt = false;
    // 高度估计先按当前板初始化，避免在 updateOutpostHeights 首次执行前
    // first_z/second_z/third_z 是未定义值，被火控当成"有效层"去排序
    first_z  = za;
    second_z = za;
    third_z  = za;
    for (int i = 0; i < 3; i++) {
        outpost_id_seen_[i] = false;
        outpost_z_seen_[i]  = {0.0, 0.0};
    }
    for (int i = 0; i < 4; i++) {
        robot_id_seen_[i] = false;
    }
    lost_thre_ = 0;
}


// ============================================================
// updateArmorsNum：根据装甲板类型更新 tracked_armors_num（逻辑不变）
// ============================================================
void Tracker::updateArmorsNum(const Armor & armor)
{
    if (tracked_armor.type == "large" &&
        (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
        tracked_armors_num = ArmorsNum::BALANCE_2;
        task_mode = 2;
    } else if (tracked_id == "outpost") {
        tracked_armors_num = ArmorsNum::OUTPOST_3;
        task_mode = 1;
    } else if (tracked_id == "base") {
        tracked_armors_num = ArmorsNum::BASE_1;
        task_mode = 3;
    } else {
        tracked_armors_num = ArmorsNum::NORMAL_4;
        task_mode = 0;
    }
}

// ============================================================
// handleArmorJump：装甲板跳变处理
// [IMM-Phase A] ekf.setState → imm.setState（同步所有子模型）
// ============================================================
void Tracker::handleArmorJump(const Armor & current_armor)
{
    double yaw = orientationToYaw(current_armor.pose.orientation);
    updateArmorsNum(current_armor);
    if (tracked_armors_num == ArmorsNum::OUTPOST_3) {
        // [思路 B] 前哨站不重置 state(6)，只重新匹配 matched_id_
        // 真正的 jump 在数据关联阶段就已经被吸收（位置匹配遍历 3 块板）
        // 这里能进入说明位置和 yaw 都不匹配 → 真异常
        std::cout << "Outpost real jump (anomaly)!" << std::endl;
        // 保守做法：清零速度，不动 yaw0
        target_state(1) = 0;
        target_state(3) = 0;
        imm->setState(target_state);
        return;
    }

    target_state(6) = yaw;
    if (tracked_armors_num == ArmorsNum::NORMAL_4) {
        first_z  = target_state(4);
        second_z = target_state(4);
        third_z  = target_state(4);
        target_state(4) = current_armor.pose.position.z;
        std::swap(target_state(8), another_r);
    }
    
    auto p = current_armor.pose.position;
    Eigen::Vector3d current_p(p.x, p.y, p.z);
    Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

    if ((current_p - infer_p).norm() > max_match_distance_) {
        target_state(1) = 0;  // vxc
        target_state(3) = 0;  // vyc
        target_state(5) = 0;  // vza
        
        RCLCPP_ERROR(rclcpp::get_logger("armor_processor"), "State wrong after jump!");
    }

    // [IMM-Phase A] 同步到所有子模型
    imm->setState(target_state);
}

// ============================================================
// orientationToYaw：四元数转偏航角（角度连续化，逻辑不变）
// ============================================================
double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    yaw = angles::normalize_angle(yaw);
    // yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
    // last_yaw_ = yaw;
    return yaw;
}

// ============================================================
// getArmorPositionFromState：从状态向量反推装甲板位置（逻辑不变）
// ============================================================
Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
{
    double xc = x(0), yc = x(2), za = x(4);
    double yaw0 = x(6), r = x(8);
    double dz = 0.0;
    if (matched_id == 1 && x.size() > 11) dz = x(11);
    else if (matched_id == 2 && x.size() > 12) dz = x(12);
    int k = getK();
    double angle = yaw0 + matched_id * (2.0 * M_PI / k);
    double xa = xc - r * cos(angle);
    double ya = yc - r * sin(angle);
    double zaa = za + dz;
    return Eigen::Vector3d(xa, ya, zaa);
}

bool Tracker::isOutpost(const jlcv_interfaces::msg::Armor & armor)
{
    return armor.number == "outpost";
}

// // ============================================================
// // matchOutpostArmorId：前哨站板号匹配
// // 用 yaw 相位预测三块板的位置，选择与观测最接近的板号
// // ============================================================
// int Tracker::matchOutpostArmorId(const Eigen::VectorXd & state, double observed_yaw, double observed_z)
// {
//     double xc = state(0), yc = state(2), za = state(4);
//     double yaw = state(6), r = state(8), dz = state(11);
//     int best_id = 0;
//     double min_error = 1e10;
//     for (int i = 0; i < 3; i++) {
//         double angle_i = yaw + i * (2.0 * M_PI / 3.0);
//         // 预测板 i 的位置
//         double xa_i = xc - r * cos(angle_i);
//         double ya_i = yc - r * sin(angle_i);
//         double za_i = za + dz * i;
//         // 方位角误差（观测板在世界坐标系下的方位角 vs 预测板的方位角）
//         // double pred_bearing = atan2(ya_i, xa_i);
//         // double obs_bearing  = atan2(observed_yaw, 1.0);  // 简化：直接用 yaw 差
//         double yaw_error = std::abs(angles::shortest_angular_distance(angle_i, observed_yaw));
//         // z 误差
//         double z_error = std::abs(observed_z - za_i);
//         // 综合误差：角度 + z（z 权重可调）
//         double total_error = yaw_error + 2.0 * z_error;
//         if (total_error < min_error) {
//             min_error = total_error;
//             best_id = i;
//         }
//     }
//     return best_id;
// }

bool Tracker::checkOutpostRebuild() const
{
    return tracked_armors_num == ArmorsNum::OUTPOST_3 &&
           outpost_id_seen_[0] && outpost_id_seen_[1] && outpost_id_seen_[2];
}

bool Tracker::checkRobotRebuild() const
{
    return tracked_armors_num == ArmorsNum::NORMAL_4 &&
           robot_id_seen_[0] && robot_id_seen_[1] && 
           robot_id_seen_[2] && robot_id_seen_[3];
    // 来改了吗？来改了吗？如果真来我就真的很欣慰了！(๑•̀ㅂ•́)و✧，毕竟之前这个条件是我写的，
    // 我怎么可能放任bug存在呢？就是简单考验一下新人而已，看看有没有动手意愿（doge）。
}

// ============================================================
// rebuildOutpost：三块板都观测到后，修正 za 和 dz 的基准
// 确保 z_i = za + dz * i 成立，za 对应 id=0 的板
// ============================================================
void Tracker::rebuildOutpost(double z_now, int & id)
{
    for (int i = 0; i < 3; i++) {
        // outpost_id_seen_[i] = false;
        std::cout << "Outpost z seen: id=" << i << ", z=" << outpost_z_seen_[i].first << ", yaw=" << outpost_z_seen_[i].second << std::endl;
    }
    std::cout << "Outpost rebuild" << std::endl;
    auto x = imm->get_x_post();
    double pre_za = x(4);
    double pre_dz = x(11);
    // 当前板的预期高度 vs 实际高度
    // double expected_z = pre_za + pre_dz * id;
    double h_now = z_now - pre_za;
    // pre_dz = outpost_z_seen_[1] - outpost_z_seen_[0];  // 以板0和板1的 z 差作为 dz 的初始估计
    // h_now = outpost_z_seen_[2] - outpost_z_seen_[1];
    // 如果 pre_dz 接近 0，说明还没估出来，无法修正
    if (std::abs(pre_dz) < 1e-4) return;

    double k = h_now / pre_dz;
    std::cout << "Rebuild check: h_now=" << h_now << ", pre_dz=" << pre_dz << ", k=" << k << std::endl;
    // 参考 TJ 代码的修正逻辑：
    // 根据 k（当前板实际偏移 / 估计dz 的比值）来判断板号归属是否正确
    if (k > 0) {
        if (pre_dz > 0) {
            std::cout << "Outpost rebuild: 分支一" << std::endl;
            std::cout << "id(修改前): " << id << std::endl; 
            // // 板0应该是z最小的，但当前za不是最小 → za需要上移，dz翻转
            // x(4) += pre_dz * 2;
            // x(6) += 2.0 * M_PI / 3.0;  // yaw 旋转120°对齐
            // x(11) *= -1;
            // id = (id - 1 + 3) % 3;
            std::cout << "id(修改后): " << id << std::endl;
        }
        if (pre_dz < 0) {
            std::cout << "Outpost rebuild: 分支四" << std::endl;
            std::cout << "dz(修改前): " << x(11) << std::endl;
            std::cout << "z(修改前): " << x(4) << std::endl;
            // // 板0应该是z最大的，但当前za不是最大 → za需要下移，dz翻转
            x(4) -= pre_dz;
            // x(6) -= 2.0 * M_PI / 3.0;  // yaw 旋转-120°对齐
            x(11) = -0.102;
            // id = (id + 1) % 3;
            std::cout << "dz(修改后): " << x(11) << std::endl;
            std::cout << "z(修改后): " << x(4) << std::endl;
        }
    }
    else {
        if (std::abs(k - (-1.0)) < std::abs(k - (-0.5))) {
            // k ≈ -1
            std::cout << "Outpost rebuild: 分支二" << std::endl;
            std::cout << "id(修改前): " << id << std::endl;
            std::cout << "z(修改前): " << x(4) << std::endl;
            std::cout << "pre_dz(修改前): " << pre_dz << std::endl;
            std::cout << "yaw(修改前): " << x(6) << std::endl;
            if (pre_dz > 0) {
                x(4) += h_now;
                x(6) -= 2.0 * M_PI / 3.0;
                // x(11) *= -1;
                id = (id + 2) % 3;
            } else {
                x(4) += h_now;
                x(6) -= 2.0 * M_PI / 3.0;
                id = (id + 2) % 3;
            }
            std::cout << "id(修改后): " << id << std::endl;
            std::cout << "pre_dz(修改后): " << x(11) << std::endl;
            std::cout << "yaw(修改后): " << x(6) << std::endl;
            std::cout << "z(修改后): " << x(4) << std::endl;
        } else {
            std::cout << "Outpost rebuild: 分支三" << std::endl;
            // k ≈ -0.5
            if (pre_dz > 0) {
                x(4) += 2*pre_dz;
                x(6) += 4.0 * M_PI / 3.0;
                x(11) *= -0.5;
                id = (id + 1) % 3;
            }
        }
    }
    imm->setState(x);
    target_state = x;
}


}  // namespace rm_auto_aim
