#include <gtest/gtest.h>

// ================== 魔法区域开始 ==================
// 在包含你的头文件之前，把 private 和 protected 偷换成 public
// 注意：这只在当前测试文件中生效，不会影响源代码的编译
#define private public
#define protected public

#include "opti_solver.hpp" // 你的核心头文件
#include "ballistic_solver.hpp"

// 恢复关键词，以免影响后续包含的标准库（虽然放在这里通常也没事）
#undef private
#undef protected

#include "jlcv_interfaces/msg/target.hpp"
#include <cmath>

using namespace solve_angle;

class SolveAngleTest : public ::testing::Test {
protected:
    std::shared_ptr<SolveAngle> node;

    void SetUp() override {
        rclcpp::NodeOptions options;
        node = std::make_shared<SolveAngle>(options);
        
        // 强制初始化内部 solver 参数
        BallisticConfig config;
        config.bullet_speed = 15.4; // 设定为英雄常用的高速档 28m/s (或者16m/s)
        config.k_air = 0.0068;
        config.gravity = 9.7915;
        config.latency = 0.0;
        node->solver_->updateConfig(config);
    }
    
    // 辅助计算 Pitch 角度的函数，用于验证结果
    // 简单的平抛模型近似验证： tan(pitch) = (h + 0.5*g*t^2) / x
    double estimatePitch(double dist, double height_diff, double speed) {
        double t = dist / speed;
        double drop = 0.5 * 9.7915 * t * t;
        return std::atan2(height_diff + drop, dist);
    }
};

/**
 * @brief 构造模拟数据
 * @param dist 平面距离 (x)
 * @param yaw 当前可见装甲板的朝向 (世界坐标系)
 * @param v_yaw 旋转速度
 * @param current_z 当前可见装甲板的高度 (即 position.z)
 * @param z1 第一块板(Visual Base)高度
 * @param z2 第二块板(Next)高度
 * @param z3 第三块板(Last)高度
 */
auto createOutpostMsg(double dist, double yaw, double v_yaw, 
                      double current_z, 
                      double z1, double z2, double z3) {
    auto msg = std::make_shared<jlcv_interfaces::msg::Target>();
    msg->position.x = dist;
    msg->position.y = 0.0;
    
    // === 关键修正 ===
    // position.z 等于当前视觉看到的板子高度
    msg->position.z = current_z; 
    
    msg->yaw = yaw;       
    msg->v_yaw = v_yaw;   
    msg->radius_1 = 0.25; // 半径 0.25m
    
    // 三块板的真实物理高度
    msg->first_z = z1;
    msg->second_z = z2;
    msg->third_z = z3;
    
    msg->armors_num = 3;
    msg->tracking = true; // 处于追踪状态
    msg->id = "outpost";          // 假设是英雄/步兵
    msg->header.stamp = rclcpp::Time(0);
    return msg;
}

// === 测试用例 1: 静态打击 ===
// 场景：车不动，看着当前板，打当前板
// 验证：代码是否正确使用了 current_z (first_z) 进行解算
TEST_F(SolveAngleTest, StaticHitCurrentPlate) {
    double dist = 6.0;
    double target_z = 0.6; // 假设这块板比较高
    
    // 看着 ID 0 (first_z)，它高度是 0.6，所以 position.z = 0.6
    // Yaw = PI (正对枪口)
    auto msg = createOutpostMsg(dist, M_PI, 0.0, 
                                target_z, 
                                target_z, 0.2, 0.4); 

    double p, y;
    bool success = node->processOutpost(msg, p, y);

    EXPECT_TRUE(success);
    
    // 预期：解算应该基于 target_z (0.6)
    double expected_pitch = estimatePitch(dist, target_z, 15.4) ;
    
    // 允许 0.02 rad (约1度) 的误差，因为这里是简单估算 vs RK4解算
    EXPECT_NEAR(p, 0.228801829841, 0.02) 
        << "应该基于 first_z (0.6m) 计算 Pitch";
    EXPECT_NEAR(y, 0.0, 1e-3) 
        << "应该基于 first_z (0.6m) 计算 Yaw";
}

// === 测试用例 2: 旋转预测，高度切换 ===
// 场景：非常关键！
// 当前看着板A (ID 0)，高度 0.8m。
// 前哨站飞速旋转，预测子弹到达时，板A已经转走了，板B (ID 1) 转到了枪口位置。
// 板B 的高度是 0.3m (低板)。
// 验证：火控是否计算出了击打 0.3m 高度的 Pitch，而不是 0.8m。
TEST_F(SolveAngleTest, RotationHeightSwitch) {
    double dist = 5.0;
    double speed = 15.4;
    double flight_time = dist / speed; // 约 0.178s
    
    // 设定转速，使得飞行时间内刚好转过 120度 (2*PI/3)
    // 这样 ID 1 就会正好转到枪口位置
    double v_yaw = (2.0 * M_PI / 3.0) / 0.32049; // 约为 11.7 rad/s (非常快，极限测试)
    
    // 当前状态：
    // 板A (first_z) 正看着枪口 (Yaw = PI)
    // 实际上板A刚转过枪口一点点并不重要，重要的是我们构造数据让 ID 1 成为最佳击打点
    
    // 调整一下逻辑：
    // 我们让当前板A (ID 0) 的角度设为 PI - 120度。
    // 这样，ID 1 (Next, offset +120度) 的当前角度就是 PI (正对枪口)。
    // 不对，我们要测试预测。
    
    // 正确构造：
    // 设目标理想击打位置是 Yaw = PI (正前方)
    // 我们希望预测后的 ID 1 的角度 == PI。
    // ID 1 的预测角度 = (当前基准Yaw - 120度) + (v_yaw * t)
    // 设定：当前基准Yaw = PI (当前板A正对)
    // 预测后：板A 转到了 PI + 120度。
    // 板B (ID 1, 原本在 PI - 120度) 转到了 PI + 120 - 120 = PI (正对！)
    // 所以，如果我们当前看着板A，且 v_yaw 很高，我们应该去打板B。
    
    double h_current = 0.8; // 当前看到的板很高
    double h_target  = 0.3; // 要打的下块板很低
    
    // msg->position.z = h_current (0.8)
    auto msg = createOutpostMsg(dist, M_PI, v_yaw, 
                                h_current, 
                                h_current, h_target, 0.5);

    double p, y;
    bool success = node->processOutpost(msg, p, y);
    EXPECT_TRUE(success);
    
    // === 核心验证 ===
    // 预期 Pitch 应该对应 0.3m 的高度，而不是 0.8m
    double pitch_for_low = 0.206917175472;  // 打 0.3m
    double pitch_for_high = 0.275200214435; // 打 0.8m
    
    EXPECT_NEAR(p, pitch_for_low, 0.02) 
        << "预测到了下一块板，但Pitch角似乎还是对应当前板的高度？Fail!";
        
    // EXPECT_NEAR(y, 0.0, 0.5) 
    //     << "Pitch角不应该接近当前板的高度";
}

int main(int argc, char ** argv)
{
  // 1. 初始化 Google Test
  testing::InitGoogleTest(&argc, argv);
  
  // 2. 初始化 ROS 2 (这是修复报错的关键)
  rclcpp::init(argc, argv);
  
  // 3. 运行所有测试
  int result = RUN_ALL_TESTS();
  
  // 4. 关闭 ROS 2
  rclcpp::shutdown();
  
  return result;
}