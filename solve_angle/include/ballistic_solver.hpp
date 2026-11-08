#ifndef BALLISTIC_SOLVER_HPP_
#define BALLISTIC_SOLVER_HPP_

#include <cmath>
#include <optional>

namespace solve_angle {

// ======================================================================
// 弹道配置
// ======================================================================
struct BallisticConfig {
    double bullet_speed;  // 弹速 (m/s)
    double gravity;       // 重力加速度 (m/s²)
    double k_air;         // 空气阻力系数 (k * v)
    double latency;       // 系统延迟 (s)
};

// ======================================================================
// 目标状态（相对坐标，x:前 y:左 z:上）
// ======================================================================
struct TargetState {
    double x, y, z;        // 位置 (m)
    double v_x, v_y, v_z;  // 速度 (m/s)
};

// ======================================================================
// 解算结果
// ======================================================================
struct SolveResult {
    double pitch;       // 输出 Pitch (rad)
    double yaw;         // 输出 Yaw   (rad)
    double fly_time;    // 预计飞行时间 (s)
    bool   is_solvable;
};

// ======================================================================
// 弹道解算器
//   - RK4 物理引擎模拟带空气阻力的弹道
//   - 割线法求 Pitch
//   - 固定点迭代求 3D 自洽解
// ======================================================================
class BallisticSolver {
public:
    explicit BallisticSolver(const BallisticConfig& config);
    void updateConfig(const BallisticConfig& config);

    /// 3D 综合解算：目标带速度时，内部固定点迭代求自洽 predict_time
    SolveResult solve(const TargetState& target);

    /// 单轴解算：给定水平距离和垂直高度，用割线法求 pitch 和 fly_time
    /// 暴露为 public 供上层 Brent 迭代直接调用
    bool solveSingleAxis(double dist_horiz, double dist_vert,
                         double& fly_time, double& pitch);

private:
    BallisticConfig config_;

    struct State {
        double x, y;
        double vx, vy;
    };

    State  getDerivative(const State& s);
    State  rk4Step(const State& s, double dt);
    double simulateTrajectory(double dist_horiz, double pitch, double& final_time);
};

}  // namespace solve_angle

#endif  // BALLISTIC_SOLVER_HPP_
