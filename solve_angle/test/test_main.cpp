#include "ballistic_solver.hpp"
#include <gtest/gtest.h>
#include <chrono>
/**
 * 测试 BallisticSolver 类的基本功能
 */
TEST(BallisticSolverTest, BasicFunction){
    solve_angle::BallisticConfig config;
    config.bullet_speed = 15.4;
    config.gravity = 9.7915;
    config.k_air = 0.0068; // 42mm 弹丸估算值
    config.latency = 0.0;

    solve_angle::BallisticSolver solver(config);
    
    solve_angle::TargetState target;
    target.x = 5.75; // 5.75米外
    target.y = 0.0;
    target.z = 0.6; // 高 0.6米
    target.v_x = 0;
    target.v_y = 0;
    target.v_z = 0;
    auto start = std::chrono::high_resolution_clock::now();
    auto result = solver.solve(target);
    auto end = std::chrono::high_resolution_clock::now();

    // 测量 solver.solve() 的执行时间（秒）
    std::chrono::duration<double> duration_sec = end - start;
    std::cout << "solve() 耗时: " << duration_sec.count() << " 秒" << std::endl;
    // 或者显示毫秒
    std::cout << "solve() 耗时: " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << " 毫秒" << std::endl;

    auto end2 = std::chrono::high_resolution_clock::now();

    // 测量两段代码之间的间隔（微秒）
    auto interval = end2 - end;
    std::cout << "代码间隔: " 
            << std::chrono::duration_cast<std::chrono::microseconds>(interval).count()
            << " 微秒" << std::endl;  // ✅ 正确单位    

    EXPECT_NEAR(result.pitch, 0.213582943598, 1e-3);
    EXPECT_NEAR(result.yaw, 0.0, 1e-6);
    EXPECT_NEAR(result.fly_time, 0.401043410465, 1e-2);

}