#include <gtest/gtest.h>
#include "ballistic_solver.hpp"
#include <cmath>

using namespace solve_angle;

class BallisticTest : public ::testing::Test {
protected:
    BallisticConfig config;
    std::unique_ptr<BallisticSolver> solver;

    void SetUp() override {
        config.bullet_speed = 15.4; // 15 m/s
        config.gravity = 9.7915;
        config.k_air = 0.0068;       // 有空气阻力
        config.latency = 0.0;
        solver = std::make_unique<BallisticSolver>(config);
    }
};

// 测试 1: 静态目标重力补偿
// 验证：有重力补偿计算出的 Pitch 应该大于 0
TEST_F(BallisticTest, GravityCompensation) {
    TargetState target;
    target.x = 5.75; // 5米外
    target.y = 0.0;
    target.z = 0.6; // 同高度
    target.v_x = 0; target.v_y = 0; target.v_z = 0;

    SolveResult res = solver->solve(target);

    EXPECT_TRUE(res.is_solvable);
    // 简单物理估算: t ≈ 0.33s, 下坠 h ≈ 0.5*9.8*0.33^2 ≈ 0.54m
    // angle ≈ atan(0.54/5) ≈ 0.1 rad ≈ 6度
    EXPECT_GT(res.pitch, 0.05); 
    EXPECT_LT(res.pitch, 0.2);
}

// 测试 2: 空气阻力影响
// 验证：开启阻力后的仰角 应该大于 关闭阻力后的仰角
TEST_F(BallisticTest, AirResistanceEffect) {
    TargetState target;
    target.x = 8.0; target.y = 0; target.z = 0;
    target.v_x = 0; target.v_y = 0; target.v_z = 0;

    // 1. 有阻力
    config.k_air = 0.0068; 
    solver->updateConfig(config);
    double pitch_with_drag = solver->solve(target).pitch;

    // 2. 无阻力
    config.k_air = 0.0;
    solver->updateConfig(config);
    double pitch_no_drag = solver->solve(target).pitch;

    EXPECT_GT(pitch_with_drag, pitch_no_drag) << "Drag should require higher pitch angle";
}
