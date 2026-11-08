#include "ballistic_solver.hpp"

namespace solve_angle {

BallisticSolver::BallisticSolver(const BallisticConfig& config) : config_(config) {}

void BallisticSolver::updateConfig(const BallisticConfig& config) {
    config_ = config;
}

// ======================================================================
// RK4 物理引擎
// ======================================================================

BallisticSolver::State BallisticSolver::getDerivative(const State& s) {
    double v = std::sqrt(s.vx * s.vx + s.vy * s.vy);

    State ds;
    ds.x  = s.vx;
    ds.y  = s.vy;

    if (v > 1e-6) {
        // 平方律阻力: F = -k * v² * v̂
        // 加速度: a = -k * v² * (v_component / v)
        double drag_acc = config_.k_air * v * v;  // 阻力加速度大小
        ds.vx = -drag_acc * s.vx / v;  // 分解到 x 方向
        ds.vy = -drag_acc * s.vy / v - config_.gravity;  // 分解到 y 方向 + 重力
    } else {
        // 速度接近零时，只有重力
        ds.vx = 0.0;
        ds.vy = -config_.gravity;
    }

    return ds;
}

BallisticSolver::State BallisticSolver::rk4Step(const State& s, double dt) {
    State k1 = getDerivative(s);

    State s2 = s;
    s2.x  += k1.x  * dt * 0.5;  s2.y  += k1.y  * dt * 0.5;
    s2.vx += k1.vx * dt * 0.5;  s2.vy += k1.vy * dt * 0.5;
    State k2 = getDerivative(s2);

    State s3 = s;
    s3.x  += k2.x  * dt * 0.5;  s3.y  += k2.y  * dt * 0.5;
    s3.vx += k2.vx * dt * 0.5;  s3.vy += k2.vy * dt * 0.5;
    State k3 = getDerivative(s3);

    State s4 = s;
    s4.x  += k3.x  * dt;  s4.y  += k3.y  * dt;
    s4.vx += k3.vx * dt;  s4.vy += k3.vy * dt;
    State k4 = getDerivative(s4);

    State next;
    next.x  = s.x  + (k1.x  + 2 * k2.x  + 2 * k3.x  + k4.x ) * dt / 6.0;
    next.y  = s.y  + (k1.y  + 2 * k2.y  + 2 * k3.y  + k4.y ) * dt / 6.0;
    next.vx = s.vx + (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx) * dt / 6.0;
    next.vy = s.vy + (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy) * dt / 6.0;
    return next;
}

double BallisticSolver::simulateTrajectory(double dist_horiz, double pitch,
                                           double& final_time) {
    const double dt = 0.001;
    State s{0.0, 0.0,
            config_.bullet_speed * std::cos(pitch),
            config_.bullet_speed * std::sin(pitch)};

    State  prev_s = s;
    double t = 0.0, prev_t = 0.0;

    while (s.x < dist_horiz && t < 3.0) {
        prev_s = s;
        prev_t = t;
        s = rk4Step(s, dt);
        t += dt;
    }

    double dx_step = s.x - prev_s.x;
    if (dx_step < 1e-6) {
        final_time = t;
        return s.y;
    }

    double alpha = (dist_horiz - prev_s.x) / dx_step;
    final_time = prev_t + alpha * dt;
    return prev_s.y + alpha * (s.y - prev_s.y);
}

// ======================================================================
// 割线法求 Pitch
// ======================================================================

bool BallisticSolver::solveSingleAxis(double dist_horiz, double dist_vert,
                                      double& fly_time, double& pitch) {
    if (dist_horiz <= 0.0) return false;

    double angle1 = std::atan2(dist_vert, dist_horiz);
    double t1;
    double h1   = simulateTrajectory(dist_horiz, angle1, t1);
    double err1 = h1 - dist_vert;

    if (std::abs(err1) < 0.001) {
        pitch    = angle1;
        fly_time = t1;
        return true;
    }

    double angle2 = angle1 + 0.03;
    double t2;
    double h2   = simulateTrajectory(dist_horiz, angle2, t2);
    double err2 = h2 - dist_vert;

    const int MAX_ITER = 15;
    for (int i = 0; i < MAX_ITER; ++i) {
        if (std::abs(err2 - err1) < 1e-9) break;

        double next_angle = angle2 - err2 * (angle2 - angle1) / (err2 - err1);
        if (next_angle > 1.4 || next_angle < -1.4) return false;

        double t_new;
        double h_new   = simulateTrajectory(dist_horiz, next_angle, t_new);
        double err_new = h_new - dist_vert;

        if (std::abs(err_new) < 0.001) {
            pitch    = next_angle;
            fly_time = t_new;
            return true;
        }

        angle1 = angle2;     err1 = err2;
        angle2 = next_angle; err2 = err_new;
    }

    pitch    = angle2;
    fly_time = t2;
    return true;
}

// ======================================================================
// 3D 综合解算：固定点迭代让 fly_time 自洽
// ======================================================================

SolveResult BallisticSolver::solve(const TargetState& target) {
    SolveResult result{};
    result.is_solvable = false;

    double pred_x = target.x;
    double pred_y = target.y;
    double pred_z = target.z;
    double pitch    = 0.0;
    double fly_time = 0.0;

    const int ITERS = 4;
    for (int i = 0; i < ITERS; ++i) {
        double dist_h = std::sqrt(pred_x * pred_x + pred_y * pred_y);
        if (!solveSingleAxis(dist_h, pred_z, fly_time, pitch)) return result;

        double total_t = fly_time + config_.latency;
        pred_x = target.x + target.v_x * total_t;
        pred_y = target.y + target.v_y * total_t;
        pred_z = target.z + target.v_z * total_t;
    }

    // 用收敛后的 pred 位置做最终解算，保证 pitch 和 yaw 基于同一预测位置
    {
        double dist_h = std::sqrt(pred_x * pred_x + pred_y * pred_y);
        if (!solveSingleAxis(dist_h, pred_z, fly_time, pitch)) return result;
    }

    result.yaw         = std::atan2(pred_y, pred_x);
    result.pitch       = pitch;
    result.fly_time    = fly_time;
    result.is_solvable = true;
    return result;
}

}  // namespace solve_angle