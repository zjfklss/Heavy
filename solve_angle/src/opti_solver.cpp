#include "opti_solver.hpp"

#include <angles/angles.h>
#include <algorithm>
#include <cmath>
#include <optional>

namespace solve_angle {

// ======================================================================
// 构造函数
// ======================================================================

SolveAngle::SolveAngle(const rclcpp::NodeOptions& options)
    : Node("solve_angle", options)
{
    // ---------- 声明参数 ----------
    this->declare_parameter("config0.bullet_speed",        15.8);
    this->declare_parameter("config0.debug_pitch_offset",  0.000);
    this->declare_parameter("config0.debug_yaw_offset",    0.000);

    this->declare_parameter("config1.bullet_speed",        15.8);
    this->declare_parameter("config1.debug_pitch_offset",  0.000);
    this->declare_parameter("config1.debug_yaw_offset",    0.000);

    this->declare_parameter("k_air_resistance", 0.0068);
    this->declare_parameter("gravity",          9.7915);
    this->declare_parameter("shoot_delay",      0.075);
    this->declare_parameter("max_yaw_error",    0.05);
    this->declare_parameter("target_frame",     "world");
    this->declare_parameter("max_linear_error", 0.06);
    this->declare_parameter("max_allow_angle",  0.25);
    this->declare_parameter("min_allow_angle",  0.015);

    // ---------- 读取参数 ----------
    solver_config_.bullet_speed = this->get_parameter("config0.bullet_speed").as_double();
    solver_config_.k_air        = this->get_parameter("k_air_resistance").as_double();
    solver_config_.gravity      = this->get_parameter("gravity").as_double();
    solver_config_.latency      = 0.0;

    params_.debug_pitch_offset = this->get_parameter("config0.debug_pitch_offset").as_double();
    params_.debug_yaw_offset   = this->get_parameter("config0.debug_yaw_offset").as_double();
    params_.max_yaw_error      = this->get_parameter("max_yaw_error").as_double();
    params_.shoot_delay        = this->get_parameter("shoot_delay").as_double();
    params_.max_linear_error   = this->get_parameter("max_linear_error").as_double();
    params_.max_allow_angle    = this->get_parameter("max_allow_angle").as_double();
    params_.min_allow_angle    = this->get_parameter("min_allow_angle").as_double();

    // ---------- 创建弹道解算器 ----------
    solver_ = std::make_unique<BallisticSolver>(solver_config_);

    // ---------- MPC Planner（可选） ----------
    this->declare_parameter("use_mpc", false);
    this->declare_parameter("mpc.max_yaw_acc",   20.0);
    this->declare_parameter("mpc.max_pitch_acc", 10.0);
    this->declare_parameter("mpc.Q_yaw",   100.0);
    this->declare_parameter("mpc.R_yaw",     1.0);
    this->declare_parameter("mpc.Q_pitch",  50.0);
    this->declare_parameter("mpc.R_pitch",   1.0);
    this->declare_parameter("mpc.fire_thresh", 0.03);

    use_mpc_ = this->get_parameter("use_mpc").as_bool();
    if (use_mpc_) {
        double max_yaw_acc   = this->get_parameter("mpc.max_yaw_acc").as_double();
        double max_pitch_acc = this->get_parameter("mpc.max_pitch_acc").as_double();
        double Q_yaw   = this->get_parameter("mpc.Q_yaw").as_double();
        double R_yaw   = this->get_parameter("mpc.R_yaw").as_double();
        double Q_pitch = this->get_parameter("mpc.Q_pitch").as_double();
        double R_pitch = this->get_parameter("mpc.R_pitch").as_double();
        double fire_thresh = this->get_parameter("mpc.fire_thresh").as_double();

        planner_ = std::make_unique<Planner>();
        planner_->setup(max_yaw_acc, max_pitch_acc,
                        Q_yaw, R_yaw, Q_pitch, R_pitch, fire_thresh);
        planner_->setEnabled(use_mpc_);

        mpc_debug_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/mpc_debug", 10);

        RCLCPP_INFO(this->get_logger(),
            "MPC Planner enabled (yaw_acc=%.1f, pitch_acc=%.1f)",
            max_yaw_acc, max_pitch_acc);
    } else {
        RCLCPP_INFO(this->get_logger(), "MPC Planner disabled");
    }

    // ---------- 订阅 ----------
    game_status_sub_ = this->create_subscription<jlcv_interfaces::msg::GameStatus>(
        "/game_status", 10,
        [this](const jlcv_interfaces::msg::GameStatus::SharedPtr msg) {
            // 短焦单相机：固定用 config0，忽略电控 camera_status
            camera_status_ = 0;
            // 电控弹速做 EMA：实测在 15.6~16.0 抖动，直接跟帧会让 pitch 跟着抖
            const float v = msg->bullet_speed;
            if (v > 10.0f && v < 20.0f) {
                if (!bullet_speed_inited_) {
                    bullet_speed_ = v;
                    bullet_speed_inited_ = true;
                } else {
                    bullet_speed_ = kBulletSpeedEmaAlpha * v +
                                    (1.0f - kBulletSpeedEmaAlpha) * bullet_speed_;
                }
            }
        });

    target_sub_ = this->create_subscription<jlcv_interfaces::msg::Target>(
        "/processor/target", rclcpp::SensorDataQoS(),
        std::bind(&SolveAngle::targetCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&SolveAngle::jointStateCallback, this, std::placeholders::_1));

    // ---------- 发布 ----------
    gimbal_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/gimbal_command", rclcpp::SensorDataQoS());

    gimbal_debug_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/gimbal_debug", rclcpp::SensorDataQoS());

    // ---------- 日志 ----------
    RCLCPP_INFO(this->get_logger(),
        "Hero Fire Control Initialized. V_Bullet: %.2f, K_Air: %.4f, ShootDelay: %.3f",
        solver_config_.bullet_speed, solver_config_.k_air, params_.shoot_delay);
    RCLCPP_INFO(this->get_logger(),
        "Debug Offsets - Pitch: %.3f, Yaw: %.3f, MaxYawError: %.3f",
        params_.debug_pitch_offset, params_.debug_yaw_offset, params_.max_yaw_error);
}

// ======================================================================
// 主回调
// ======================================================================

void SolveAngle::targetCallback(const jlcv_interfaces::msg::Target::SharedPtr msg) {
    three_z.clear();
    // 短焦单相机：始终使用 config0
    camera_status_ = 0;
    const double yaml_bullet_speed = this->get_parameter("config0.bullet_speed").as_double();
    solver_config_.bullet_speed = bullet_speed_inited_
                                      ? static_cast<double>(bullet_speed_)
                                      : yaml_bullet_speed;
    solver_config_.k_air        = this->get_parameter("k_air_resistance").as_double();
    params_.debug_pitch_offset  = this->get_parameter("config0.debug_pitch_offset").as_double();
    params_.debug_yaw_offset    = this->get_parameter("config0.debug_yaw_offset").as_double();

    three_z.push_back(msg->first_z);
    three_z.push_back(msg->second_z);
    three_z.push_back(msg->third_z);

    // --- 目标丢失处理 ---
    if (!msg->tracking) {
        shoot_flag_ = 0;
        lock_plate_ = -1;   // 目标丢失后重新选板，避免沿用旧板号
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Target lost, holding gimbal (no absolute 0/0 command).");
        publishHoldCommand();
        return;
    }
    // RCLCPP_INFO(this->get_logger(), "Target get, ready to fire.");

    // --- 动态延迟计算 ---
    rclcpp::Time now          = this->now();
    rclcpp::Time target_stamp = msg->header.stamp;
    double pipeline_delay     = (now - target_stamp).seconds();

    if (pipeline_delay < 0.0 || pipeline_delay > 0.5) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Abnormal pipeline latency: %.4fs, fallback to 0.01s", pipeline_delay);
        pipeline_delay = 0.01;
    }

    double total_latency     = pipeline_delay + params_.shoot_delay;
    solver_config_.latency   = total_latency;
    solver_->updateConfig(solver_config_);
    RCLCPP_DEBUG(this->get_logger(), "Bullet Speed is %f", solver_config_.bullet_speed);

    // --- 每帧重置开火建议 ---
    suggest_fire_ = false;

    // --- 状态机分发 ---
    double final_pitch = 0.0;
    double final_yaw   = 0.0;
    bool   solvable    = false;

    bool is_outpost        = (msg->armors_num == 3) && (std::abs(msg->v_yaw) >= 0.5);
    bool is_outpost_static = (msg->armors_num == 3) && (std::abs(msg->v_yaw) < 0.5);
    bool is_assessment_mode = (msg->armors_num == 4);


    if (is_outpost) {
        // std::cout << "outpost!!!" << std::endl;
        solvable = processOutpost(msg, final_pitch, final_yaw);
    } else if (is_outpost_static){
        // std::cout << "static outpost!!!" << std::endl;
        solvable = processOutpostStatic(msg, final_pitch, final_yaw);
    } else if (is_assessment_mode) {
        // std::cout << "infantry!!!" << std::endl;
        solvable = processAssessment2(msg, final_pitch, final_yaw);
    } else {
        solvable = processTracking(msg, final_pitch, final_yaw);
        suggest_fire_ = solvable;
    }

    // --- MPC 平滑（可选） ---
    if (solvable && use_mpc_ && planner_ && planner_->isEnabled()) {
        // 生成参考轨迹
        Trajectory yaw_ref, pitch_ref;
        generateMPCReference(msg, params_.debug_pitch_offset, params_.debug_yaw_offset,
                             yaw_ref, pitch_ref);

        // 当前帧云台角速度（用一阶差分的 v_yaw 和 0 近似 pitch vel）
        double current_yaw_vel   = msg->v_yaw;
        double current_pitch_vel = 0.0;

        Plan mpc_plan = planner_->plan(
            final_yaw, current_yaw_vel,
            final_pitch, current_pitch_vel,
            yaw_ref, pitch_ref);

        if (mpc_plan.valid) {
            // 用 MPC 平滑结果替换原始角度
            final_yaw   = mpc_plan.yaw;
            final_pitch = mpc_plan.pitch;

            // 用 MPC 跟踪误差作为开火判据（优先级高于 suggest_fire_）
            if (suggest_fire_) {
                suggest_fire_ = mpc_plan.fire_error < this->get_parameter("mpc.fire_thresh").as_double();
            }

            // 发布 MPC 调试轨迹（可视化）
            if (mpc_debug_pub_) {
                visualization_msgs::msg::Marker traj_marker;
                traj_marker.header.frame_id = "world";
                traj_marker.header.stamp = this->now();
                traj_marker.ns = "mpc_traj";
                traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                traj_marker.scale.x = 0.02;
                traj_marker.color.a = 0.8;
                traj_marker.color.b = 1.0;
                traj_marker.points.clear();
                for (int i = 0; i < MPC_HORIZON; i += 5) {
                    geometry_msgs::msg::Point p;
                    p.x = i * MPC_DT;
                    p.y = yaw_ref(0, i);
                    p.z = pitch_ref(0, i);
                    traj_marker.points.push_back(p);
                }
                mpc_debug_pub_->publish(traj_marker);
            }
        }
    }

    // --- 发布指令 ---
    if (solvable) {
        final_pitch += params_.debug_pitch_offset;
        final_yaw   += params_.debug_yaw_offset;
        // 记录最近一次有效指令，供丢失/解算失败时保持云台
        has_last_cmd_   = true;
        last_pitch_cmd_ = final_pitch;
        last_yaw_cmd_   = final_yaw;
        last_cmd_stamp_ = this->now();

        shoot_flag_  = suggest_fire_ ? 2 : 1;

        // 角度偏差过大时仅校准，不开火
        // std::cout <<"v_yaw: "<< msg->v_yaw <<" vx: " << msg->velocity.x << "vy: " << msg->velocity.y << std::endl; 
        // if((abs(msg->v_yaw) < 0.1 && abs(msg->velocity.x) < 0.1) && abs(msg->velocity.y) < 0.1){
        //     if (std::abs(angles::shortest_angular_distance(final_pitch, curr_pitch)) > 0.02 ||
        //         std::abs(angles::shortest_angular_distance(final_yaw, curr_yaw)) > 0.1) {
        //         shoot_flag_ = 1;
        //         std::cout << "Too big Diff NO FIRE!!!" << std::endl;
        //     }
        // }
        

        publishCommand(final_pitch, final_yaw, shoot_flag_);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Unsolved target, holding gimbal.");
        shoot_flag_ = 0;
        publishHoldCommand();
    }
}

// ======================================================================
// 关节状态回调
// ======================================================================

void SolveAngle::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "pitch_joint") {
            curr_pitch = msg->position[i];
        } else if (msg->name[i] == "yaw_joint") {
            curr_yaw = msg->position[i];
        }
    }
}

// ======================================================================
// 普通追踪（不旋转 / 不满足考核模式条件）
// ======================================================================

bool SolveAngle::processTracking(const jlcv_interfaces::msg::Target::SharedPtr msg,
                                 double& pitch_out, double& yaw_out) {
    TargetState t;
    t.x   = msg->position.x;  t.y   = msg->position.y;  t.z   = msg->position.z;
    t.v_x = msg->velocity.x;  t.v_y = msg->velocity.y;  t.v_z = msg->velocity.z;

    SolveResult res = solver_->solve(t);
    if (res.is_solvable) {
        pitch_out = res.pitch;
        yaw_out   = res.yaw;
        return true;
    }
    return false;
}

// ======================================================================
// Brent 二分法求自洽 predict_time（公共逻辑提取）
//
// 求解方程：f(T) = ToF(d(T)) + latency - T = 0
//   d(T) = sqrt((x0 + vx*T)² + (y0 + vy*T)²)
//   ToF(d) 由 solveSingleAxis 给出
// ======================================================================

std::optional<double> SolveAngle::solvePredictTime(double cur_x, double cur_y,
                                                   double cur_z, double vx, double vy) {
    // 残差函数
    auto computeResidual = [&](double T) -> std::optional<double> {
        double px     = cur_x + vx * T;
        double py     = cur_y + vy * T;
        double dist_h = std::sqrt(px * px + py * py);
        if (dist_h < 0.1) return std::nullopt;

        double fly_t = 0.0, pitch_tmp = 0.0;
        if (!solver_->solveSingleAxis(dist_h, cur_z, fly_t, pitch_tmp))
            return std::nullopt;

        return fly_t + solver_config_.latency - T;
    };

    // 确定搜索区间
    double dist_cur  = std::sqrt(cur_x * cur_x + cur_y * cur_y);
    double speed_tgt = std::hypot(vx, vy);
    double T_lo      = solver_config_.latency;
    double T_hi      = (dist_cur + speed_tgt * 2.5) /
                       (solver_config_.bullet_speed * 0.75) + solver_config_.latency;
    T_hi = std::clamp(T_hi, T_lo + 0.05, 2.5);

    auto r_lo_opt = computeResidual(T_lo);
    auto r_hi_opt = computeResidual(T_hi);
    if (!r_lo_opt || !r_hi_opt) return std::nullopt;

    double r_lo = *r_lo_opt;
    double r_hi = *r_hi_opt;
    double predict_time = T_hi;

    if (r_lo * r_hi > 0.0) {
        // 同号：区间内无零点，退化取残差较小端点
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[Brent] No sign change [%.3f,%.3f] r=[%.4f,%.4f], using fallback",
            T_lo, T_hi, r_lo, r_hi);
        predict_time = (std::abs(r_lo) < std::abs(r_hi)) ? T_lo : T_hi;
    } else {
        // 正常二分迭代
        double T_a = T_lo, T_b = T_hi;
        double f_a = r_lo, f_b = r_hi;

        for (int iter = 0; iter < 12; ++iter) {
            double T_mid     = 0.5 * (T_a + T_b);
            auto   r_mid_opt = computeResidual(T_mid);
            if (!r_mid_opt) break;
            double f_mid = *r_mid_opt;

            if (std::abs(f_mid) < 5e-5 || (T_b - T_a) < 1e-5) {
                predict_time = T_mid;
                break;
            }

            if (f_a * f_mid < 0.0) { T_b = T_mid; f_b = f_mid; }
            else                    { T_a = T_mid; f_a = f_mid; }

            predict_time = T_mid;
        }
    }

    return predict_time;
}

// ======================================================================
// 前哨站（定头伏击最高板：完美补偿平移 + 免疫旋转抖动）
// ======================================================================
bool SolveAngle::processOutpost(const jlcv_interfaces::msg::Target::SharedPtr msg,
                                double& pitch_out, double& yaw_out) {
    // ------------------------------------------------------------------
    // 1. 获取基础数据
    // ------------------------------------------------------------------
    double r     = (msg->radius_1 > 0.1) ? msg->radius_1 : 0.25;
    double cur_x = msg->position.x;
    double cur_y = msg->position.y;
    double vx    = msg->velocity.x;
    vx = 0.0;
    double vy    = msg->velocity.y;
    vy = 0.0;
    // ------------------------------------------------------------------
    // 2. 三层高度数据提取与验证
    // ------------------------------------------------------------------
    struct ArmorLayer {
        double z;           // 高度
        double offset;      // 相位偏移（0, 120°, 240°）
        int    index;       // 原始索引（0/1/2）
        bool   valid;       // 数据有效性
    };

    ArmorLayer layers[3] = {
        {msg->first_z,  0.0,                       0, abs(msg->first_z)  > 0.01},
        {msg->second_z, 2.0 * M_PI / 3.0,   1, abs(msg->second_z) > 0.01},
        {msg->third_z,  4.0 * M_PI / 3.0,   2, abs(msg->third_z)  > 0.01}
    };
    // 统计有效层数
    int valid_count = 0;
    for (int i = 0; i < 3; ++i) {
        if (layers[i].valid) valid_count++;
    }
    // 如果三层数据不全，降级到简化逻辑
    if (valid_count < 2) {
        RCLCPP_WARN(this->get_logger(),
            "Outpost layer data incomplete (%d/3), using fallback", valid_count);
        return processOutpostFallback(msg, pitch_out, yaw_out);
    }
    // ------------------------------------------------------------------
    // 3. 选择目标层：始终打最高有效层
    //    旧逻辑在"最高层与次高层高度差 < 0.04m"时按当前 pitch 就近选层，
    //    该分支一旦生效会形成正反馈（云台压低后更倾向选低层），把目标层
    //    锁在下面的板上，表现为"有时锁到下面的板"。三层高度差实际约 0.1m，
    //    收敛后最高层稳定，直接固定打最高有效层即可。
    // ------------------------------------------------------------------
    std::sort(std::begin(layers), std::end(layers),
              [](const ArmorLayer& a, const ArmorLayer& b) {
                  if (a.valid != b.valid) return a.valid > b.valid;
                  return a.z > b.z;
              });
    const int target_layer_idx = 0;
    ArmorLayer& target_layer = layers[target_layer_idx];
    // ------------------------------------------------------------------
    // 4. 使用 Brent 法求自洽预测时间
    // ------------------------------------------------------------------
    // 生成正对我们的虚拟平移点（不含旋转半径）
    double center_yaw = std::atan2(cur_y, cur_x);
    double virtual_x  = cur_x - r * std::cos(center_yaw);
    double virtual_y  = cur_y - r * std::sin(center_yaw);
    auto predict_opt = solvePredictTime(virtual_x, virtual_y, target_layer.z, vx, vy);
    if (!predict_opt) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Outpost: predict_time solve failed");
        return false;
    }
    double predict_time = *predict_opt;
    // ------------------------------------------------------------------
    // 5. 计算预测时刻的稳定伏击点
    // ------------------------------------------------------------------
    double pred_cx = cur_x + vx * predict_time;
    double pred_cy = cur_y + vy * predict_time;
    double aim_yaw = std::atan2(cur_y, cur_x);
    // 最终弹道靶点：预测中心的表面正前方，目标层高度
    TargetState ambush_target;
    ambush_target.x   = pred_cx - r * std::cos(aim_yaw);
    ambush_target.y   = pred_cy - r * std::sin(aim_yaw);
    ambush_target.z   = target_layer.z;
    ambush_target.v_x = 0.0;
    ambush_target.v_y = 0.0;
    ambush_target.v_z = 0.0;
    SolveResult res = solver_->solve(ambush_target);
    if (!res.is_solvable) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Outpost: ballistic solve failed");
        return false;
    }
    pitch_out = res.pitch;
    yaw_out   = aim_yaw;
    // ------------------------------------------------------------------
    // 6. 精准开火判定：目标层装甲板经过伏击点时开火
    // ------------------------------------------------------------------
    // 当前目标层装甲板的真实朝向
    double target_plate_current_yaw;
    double yaw0 = msg->yaw + msg->match_id * 2 * M_PI / 3;
    // double v_yaw = 1.6*M_PI - abs(msg->v_yaw);
    if(msg->v_yaw > 0){
        target_plate_current_yaw = yaw0 - target_layer.offset;
    } else{
        target_plate_current_yaw = yaw0 + target_layer.offset;
    }
    // 预测子弹到达时的朝向
    double target_plate_pred_yaw = target_plate_current_yaw + msg->v_yaw * predict_time;
    // 计算预测朝向与枪管指向的夹角
    double angular_diff = std::abs(angles::shortest_angular_distance(target_plate_pred_yaw, aim_yaw));
    // 开火窗口策略：
    // - 高速旋转（|v_yaw| > 2.0 rad/s）：窄窗口 4°（精准狙击）
    // - 中速旋转（0.5 < |v_yaw| <= 2.0）：中等窗口 8°
    // - 低速/静止（|v_yaw| <= 0.5）：宽窗口 15°（快速击杀）
    double abs_v_yaw = std::abs(msg->v_yaw);
    double fire_window;
    
    if (abs_v_yaw > 2.0) {
        fire_window = 8.0 * M_PI / 180.0;
    } else if (abs_v_yaw > 0.5) {
        fire_window = 12.0 * M_PI / 180.0;
    } else {
        fire_window = 15.0 * M_PI / 180.0;
    }
    // 额外条件：距离过近时放宽窗口（避免近距离miss）
    double distance = std::hypot(pred_cx, pred_cy);
    if (distance < 3.0) {
        fire_window *= 1.5;
    }
    // 最终开火判定（target_layer 已保证 valid）
    bool is_aligned = (angular_diff < fire_window);
    bool is_stable  = (abs_v_yaw < 0.3) && (angular_diff < fire_window * 2.0);  // 静止容错
    
    suggest_fire_ = (is_aligned || is_stable) && target_layer.valid;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "OutpostSpin: layer=%d z=%.2f v_yaw=%.2f diff=%.1f° window=%.1f° fire=%d",
        target_layer.index, target_layer.z, msg->v_yaw,
        angular_diff * 180.0 / M_PI, fire_window * 180.0 / M_PI, suggest_fire_);
    return true;
}

bool SolveAngle::processOutpostFallback(const jlcv_interfaces::msg::Target::SharedPtr msg,
                                        double& pitch_out, double& yaw_out) {
    // 使用当前观测到的装甲板位置直接解算
    double r     = (msg->radius_1 > 0.1) ? msg->radius_1 : 0.25;
    double cur_x = msg->position.x;
    double cur_y = msg->position.y;
    double cur_z = msg->position.z;
    // 三层高度数据不全时，仍优先打已知的最高层板，避免退化为"当前锁到哪块打哪块"
    {
        double best_z = -1e9;
        for (double z : {msg->first_z, msg->second_z, msg->third_z}) {
            if (std::abs(z) > 0.01 && z > best_z) best_z = z;
        }
        if (best_z > -1e8) cur_z = best_z;
    }
    double vx    = msg->velocity.x;
    vx = 0.0;
    double vy    = msg->velocity.y;
    vy = 0.0;
    // 简化预测：只考虑平移，不考虑旋转
    double center_yaw = std::atan2(cur_y, cur_x);
    double virtual_x  = cur_x - r * std::cos(center_yaw);
    double virtual_y  = cur_y - r * std::sin(center_yaw);
    auto predict_opt = solvePredictTime(virtual_x, virtual_y, cur_z, vx, vy);
    if (!predict_opt) return false;
    double predict_time = *predict_opt;
    double pred_cx = cur_x + vx * predict_time;
    double pred_cy = cur_y + vy * predict_time;
    double aim_yaw = std::atan2(pred_cy, pred_cx);
    TargetState target;
    target.x   = pred_cx - r * std::cos(aim_yaw);
    target.y   = pred_cy - r * std::sin(aim_yaw);
    target.z   = cur_z;
    target.v_x = 0.0;
    target.v_y = 0.0;
    target.v_z = 0.0;
    SolveResult res = solver_->solve(target);
    if (!res.is_solvable) return false;
    pitch_out = res.pitch;
    yaw_out   = aim_yaw;
    // 降级模式：更保守的开火策略
    double angular_diff = std::abs(angles::shortest_angular_distance(msg->yaw, aim_yaw));
    suggest_fire_ = (angular_diff < 12.0 * M_PI / 180.0) && (std::abs(msg->v_yaw) < 1.0);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Outpost: using fallback mode");
    return true;
}

bool SolveAngle::processOutpostStatic(const jlcv_interfaces::msg::Target::SharedPtr msg,
                                    double& pitch_out, double& yaw_out) {
    double r     = msg->radius_1;
    double cur_x = msg->position.x;
    double cur_y = msg->position.y;
    double cur_z = msg->first_z;
    double vx    = msg->velocity.x;
    vx = 0.0;
    double vy    = msg->velocity.y;
    vy = 0.0;
    // --- Brent 法求自洽 predict_time ---
    auto predict_opt = solvePredictTime(cur_x, cur_y, cur_z, vx, vy);
    if (!predict_opt) return false;
    double predict_time = *predict_opt;

    // --- 预测中心位置 ---
    double pred_cx = cur_x + vx * predict_time;
    double pred_cy = cur_y + vy * predict_time;
    double pred_cz = cur_z;

    // --- 选最佳装甲板 ---
    double yaw0 = angles::normalize_angle(msg->yaw + msg->match_id * 2.0 * M_PI / 3.0);
    double pred_yaw_base   = yaw0 + msg->v_yaw * predict_time;
    double best_armor_yaw = pred_yaw_base;

    // --- 弹道解算（速度置零，避免重复叠加） ---
    TargetState final_target;
    final_target.x   = pred_cx - r * std::cos(best_armor_yaw);
    final_target.y   = pred_cy - r * std::sin(best_armor_yaw);
    final_target.z   = pred_cz;
    final_target.v_x = 0.0;
    final_target.v_y = 0.0;
    final_target.v_z = 0.0;

    SolveResult res = solver_->solve(final_target);
    if (!res.is_solvable) return false;

    // --- 开火判定 ---
    suggest_fire_ = true;
    geometry_msgs::msg::Vector3 cmd;
    cmd.x = final_target.x;
    cmd.y = final_target.y;
    cmd.z = 0;
    gimbal_debug_pub_->publish(cmd);

    if(suggest_fire_ == 0){
        RCLCPP_INFO(this->get_logger(),
            "Outpost:  final_z = %.2f yaw=%.1f° fire=%d",
            final_target.z, best_armor_yaw * 180.0 / M_PI, suggest_fire_);
    }


    pitch_out = res.pitch;
    std::cout << "Pitch Out: " << pitch_out << std::endl;
    yaw_out   = std::atan2(final_target.y, final_target.x);
    return true;


}
// ======================================================================
// 考核模式：旋转 + 平移目标（4 块板，含入射角判定）
// ======================================================================

bool SolveAngle::processAssessment(const jlcv_interfaces::msg::Target::SharedPtr msg,
                                   double& pitch_out, double& yaw_out) {
    double r     = msg->radius_1;
    double cur_x = msg->position.x;
    double cur_y = msg->position.y;
    double cur_z = msg->position.z;
    double vx    = msg->velocity.x;
    double vy    = msg->velocity.y;
    double axc   = msg->a_xc;
    double ayc   = msg->a_yc;

    // --- 掉头检测 ---
    double dot    = vx * last_vx_ + vy * last_vy_;
    is_turning_   = dot < -0.5;
    last_vx_      = vx;
    last_vy_      = vy;

    // --- Brent 法求自洽 predict_time ---
    auto predict_opt = solvePredictTime(cur_x, cur_y, cur_z, vx, vy);
    if (!predict_opt) return false;
    double predict_time = *predict_opt;

    // --- 预测中心位置 ---
    double pred_cx = cur_x + vx * predict_time;
    double pred_cy = cur_y + vy * predict_time;
    double pred_cz = cur_z;

    // --- 选最佳装甲板 ---
    double center_view_yaw = std::atan2(pred_cy, pred_cx);
    double pred_yaw_base   = msg->yaw + msg->v_yaw * predict_time;
    double armor_spacing   = 2.0 * M_PI / msg->armors_num;

    int    best_idx = 0;
    double min_diff = 1e6;
    for (int i = -1; i < 3; ++i) {
        double this_yaw = pred_yaw_base + i * armor_spacing;
        double diff     = std::abs(angles::shortest_angular_distance(
                              this_yaw, center_view_yaw + M_PI));
        if (diff < min_diff) { min_diff = diff; best_idx = i; }
    }
    double best_armor_yaw = pred_yaw_base + best_idx * armor_spacing;

    // --- 弹道解算（速度置零，避免重复叠加） ---
    TargetState final_target;
    final_target.x   = pred_cx - r * std::cos(best_armor_yaw);
    final_target.y   = pred_cy - r * std::sin(best_armor_yaw);
    final_target.z   = pred_cz;
    final_target.v_x = 0.0;
    final_target.v_y = 0.0;
    final_target.v_z = 0.0;

    SolveResult res = solver_->solve(final_target);
    if (!res.is_solvable) return false;

    pitch_out = res.pitch;
    yaw_out   = res.yaw;

    // --- 3D 入射角判定 ---
    // 子弹初始速度方向（单位向量）
    double cos_p = std::cos(res.pitch);
    double b_x   = cos_p * std::cos(res.yaw);
    double b_y   = cos_p * std::sin(res.yaw);

    // 装甲板法向量（竖直安装，无 Z 分量）
    double n_x = std::cos(best_armor_yaw);
    double n_y = std::sin(best_armor_yaw);

    // 入射角
    double dot_bn = -(b_x * n_x + b_y * n_y);
    dot_bn = std::clamp(dot_bn, -1.0, 1.0);
    double incidence_angle = std::acos(dot_bn);

    // --- 开火判定 ---
    const double MAX_FACE_ERR       = 0.15;
    const double MAX_INCIDENCE_ANGLE = 0.15;  // ~8.6°

    suggest_fire_ = (!is_turning_)
                 && (min_diff < MAX_FACE_ERR)
                 && (incidence_angle < MAX_INCIDENCE_ANGLE);

    return true;
}

// ======================================================================
// 考核模式 2：三板简化版（无入射角判定）
// ======================================================================

bool SolveAngle::processAssessment2(const jlcv_interfaces::msg::Target::SharedPtr msg,
                                    double& pitch_out, double& yaw_out) {
    double r     = msg->radius_1;
    double cur_x = msg->position.x;
    double cur_y = msg->position.y;
    double cur_z = msg->position.z;
    double vx    = msg->velocity.x;
    double vy    = msg->velocity.y;

    // --- 掉头检测 ---
    double dot    = vx * last_vx_ + vy * last_vy_;
    is_turning_   = dot < -0.5;
    last_vx_      = vx;
    last_vy_      = vy;

    // --- Brent 法求自洽 predict_time ---
    auto predict_opt = solvePredictTime(cur_x, cur_y, cur_z, vx, vy);
    if (!predict_opt) return false;
    double predict_time = *predict_opt;

    // --- 预测中心位置 ---
    double pred_cx = cur_x + vx * predict_time;
    double pred_cy = cur_y + vy * predict_time;
    double pred_cz = cur_z;

    // --- 选最佳装甲板（带锁板防抖）---
    double center_view_yaw = std::atan2(pred_cy, pred_cx);
    double yaw0 = angles::normalize_angle(msg->yaw + msg->match_id * 2.0 * M_PI / 4.0);
    double pred_yaw_base   = yaw0 + msg->v_yaw * predict_time;
    double armor_spacing   = 2.0 * M_PI / 4;

    int    best_idx   = 0;
    double min_diff   = 1e6;
    int    best_plate = -1;     // 物理板号 (0..3)，跨帧不变，用于锁板
    int    lock_idx   = -999;   // 上一帧锁定板在本帧的候选序号
    double lock_diff  = 1e6;
    for (int i = -1; i < 3; ++i) {
        double this_yaw = pred_yaw_base + i * armor_spacing;
        double diff     = std::abs(angles::shortest_angular_distance(this_yaw, center_view_yaw));
        int    plate    = ((msg->match_id + i) % 4 + 4) % 4;
        if (diff < min_diff) { min_diff = diff; best_idx = i; best_plate = plate; }
        if (lock_plate_ == plate) { lock_idx = i; lock_diff = diff; }
    }

    // 上一帧锁定的板仍接近最佳板（容差 5°）时保持锁定，
    // 避免目标旋转到两块板呈对称角度时在“正对板/侧板”之间来回切换
    if (lock_idx != -999 && lock_diff < min_diff + 5.0 * M_PI / 180.0) {
        best_idx   = lock_idx;
        min_diff   = lock_diff;
        best_plate = lock_plate_;
    }
    lock_plate_ = best_plate;

    double best_armor_yaw = pred_yaw_base + best_idx * armor_spacing;

    // --- 弹道解算（速度置零，避免重复叠加） ---
    TargetState final_target;
    final_target.x   = pred_cx - r * std::cos(best_armor_yaw);
    final_target.y   = pred_cy - r * std::sin(best_armor_yaw);
    // if(msg->v_yaw > 0){
    //     if(best_idx == -1){
    //         final_target.z = msg->second_z;
    //     } else if(best_idx == 1){
    //         final_target.z = msg->third_z;
    //     } else if(best_idx == 0){
    //         final_target.z = msg->first_z;
    //     } else {
    //         final_target.z = msg->first_z;
    //     }
    // } else{
    //     if(best_idx == 1){
    //         final_target.z = msg->second_z;
    //     } else if(best_idx == -1){
    //         final_target.z = msg->third_z;
    //     } else if(best_idx == 0){
    //         final_target.z = msg->first_z;
    //     } else {
    //         final_target.z = msg->first_z;
    //     }
    // }
    double max_z = *max_element(three_z.begin(), three_z.end());
    double min_z = *min_element(three_z.begin(), three_z.end());
    // std::cout << "fin z: " << final_target.z << std::endl;
    // std::cout << "max_z: " << max_z << std::endl;
    // std::cout << "min_z: " << min_z << std::endl;
    final_target.z   = pred_cz;
    final_target.v_x = 0.0;
    final_target.v_y = 0.0;
    final_target.v_z = 0.0;

    SolveResult res = solver_->solve(final_target);
    if (!res.is_solvable) return false;

    pitch_out = res.pitch;
    if(abs(msg->v_yaw) >= 1){
        yaw_out   = center_view_yaw;
    } else{
        yaw_out   = std::atan2(final_target.y, final_target.x);
    }
    

    // --- 开火判定 ---
    double threshold       = 4.0 * M_PI / 180.0;
    // 注意：必须用 |v_yaw| 判断，原来写的是 msg->v_yaw < 1，
    // 会让逆时针旋转（v_yaw<0）永远命中"低速放宽"分支，开火窗口比顺时针大一个数量级
    bool   is_low_spinning = (std::abs(msg->v_yaw) < 1) && (min_diff < 10 * threshold);
    suggest_fire_ = (min_diff < threshold || is_low_spinning);//&& ;
    geometry_msgs::msg::Vector3 cmd;
    cmd.x = final_target.z;
    cmd.y = yaw0;
    cmd.z = best_idx;
    gimbal_debug_pub_->publish(cmd);

    if(suggest_fire_ == 0){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Assessment4: final_z = %.2f diff=%.1f° window=%.1f° fire=%d",
            final_target.z, min_diff * 180.0 / M_PI, threshold * 180.0 / M_PI, suggest_fire_);
    }


    return true;
}

// ======================================================================
// 辅助：calculateHitPoint
// ======================================================================

SolveAngle::HitPoint SolveAngle::calculateHitPoint(
    const jlcv_interfaces::msg::Target::SharedPtr msg,
    double predict_time, int target_idx, double z_aim) {
    double r = (msg->radius_1 > 0.1) ? msg->radius_1 : 0.25;
    double cur_yaw  = msg->yaw;
    double center_x = msg->position.x - r * std::cos(cur_yaw) + msg->velocity.x * predict_time;
    double center_y = msg->position.y - r * std::sin(cur_yaw) + msg->velocity.y * predict_time;

    double armor_spacing = 2.0 * M_PI / msg->armors_num;
    double aim_yaw       = cur_yaw + msg->v_yaw * predict_time + target_idx * armor_spacing;

    HitPoint hit;
    hit.x   = center_x + r * std::cos(aim_yaw);
    hit.y   = center_y + r * std::sin(aim_yaw);
    hit.z   = z_aim;
    hit.yaw = aim_yaw;
    return hit;
}

// ======================================================================
// 辅助：publishCommand
// ======================================================================

void SolveAngle::publishCommand(double pitch, double yaw, int shoot_flag) {
    geometry_msgs::msg::Vector3 cmd;
    cmd.x = pitch;
    cmd.y = yaw;
    cmd.z = static_cast<double>(shoot_flag);
    gimbal_cmd_pub_->publish(cmd);
}

// ======================================================================
// 辅助：publishHoldCommand
// 目标丢失 / 弹道解算失败时调用。绝不发送绝对 (0,0)：
// 之前版本发 (0,0,0) 会让云台直接跳到水平角，表现为"莫名低头"
// ======================================================================

void SolveAngle::publishHoldCommand() {
    bool use_last = has_last_cmd_ &&
        (this->now() - last_cmd_stamp_).seconds() < cmd_hold_timeout_;
    if (use_last) {
        // 最近一次有效指令仍新鲜：原样保持（仅关火），避免短暂丢帧导致云台跳变
        publishCommand(last_pitch_cmd_, last_yaw_cmd_, 0);
    }
    // 超时后不再发送指令：交给电控自行保持/超时，既不低头，也不和手动控云台打架
}

// ======================================================================
// 辅助：动态开火阈值
// ======================================================================

double SolveAngle::getDynamicThreshold(double distance) {
    if (distance < 0.1) return params_.max_allow_angle;
    double threshold = std::atan2(params_.max_linear_error, distance);
    return std::clamp(threshold, params_.min_allow_angle, params_.max_allow_angle);
}

// ======================================================================
// MPC 参考轨迹生成
// 基于目标匀速模型 + 弹道解算，生成未来 1s 的参考 (yaw, pitch) 序列
// ======================================================================

void SolveAngle::generateMPCReference(
    const jlcv_interfaces::msg::Target::SharedPtr msg,
    double pitch_offset, double yaw_offset,
    Trajectory& yaw_ref, Trajectory& pitch_ref)
{
    double vx = msg->velocity.x;
    double vy = msg->velocity.y;
    double r  = msg->radius_1;

    for (int i = 0; i < MPC_HORIZON; ++i) {
        double dt = i * MPC_DT;

        // 预测目标中心位置（匀速模型）
        double pred_cx = msg->position.x + vx * dt;
        double pred_cy = msg->position.y + vy * dt;

        // 预测装甲板朝向
        double pred_yaw_angle = msg->yaw + msg->v_yaw * dt;

        // 装甲板在目标表面上的位置
        double pred_ax = pred_cx - r * std::cos(pred_yaw_angle);
        double pred_ay = pred_cy - r * std::sin(pred_yaw_angle);
        double pred_az = msg->position.z;  // 假设高度不变

        // Yaw 参考：从枪管到预测点（不含 offset，offset 在输出时统一加）
        double yaw_i = std::atan2(pred_ay, pred_ax);
        yaw_ref(0, i) = yaw_i;
        yaw_ref(1, i) = msg->v_yaw;  // 参考角速度

        // Pitch 参考：用弹道解算（不含 offset）
        double dist_h = std::sqrt(pred_ax * pred_ax + pred_ay * pred_ay);
        double fly_t  = 0.0, pitch_i = 0.0;
        if (dist_h > 0.1) {
            solver_->solveSingleAxis(dist_h, pred_az, fly_t, pitch_i);
        }
        pitch_ref(0, i) = -pitch_i;
        pitch_ref(1, i) = 0.0;
    }
}

}  // namespace solve_angle

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(solve_angle::SolveAngle)
