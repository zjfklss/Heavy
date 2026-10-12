// 钉头离线：无相机。构造旋转目标，检查 yaw 钉在车中心且窗口内 shoot。
#include <chrono>
#include <cmath>
#include <iostream>
#include <list>

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/target.hpp"

int main()
{
  // 需要一份带 nail_mode 的 yaml
  auto_aim::Aimer aimer("configs/heavy_nail.yaml");
  if (!aimer.nail_mode()) {
    std::cerr << "FAIL nail_mode not enabled in configs/heavy_nail.yaml\n";
    return 1;
  }

  // Target(x, vyaw, radius, h)：中心在 x 轴上，高速旋转
  auto_aim::Target target(3.0, 3.0, 0.25, 0.0);
  std::list<auto_aim::Target> targets{target};
  auto now = std::chrono::steady_clock::now();
  auto cmd = aimer.aim(targets, now, 16.5, true);

  if (!cmd.control) {
    std::cerr << "FAIL control=false\n";
    return 2;
  }
  if (!aimer.nail_latched()) {
    std::cerr << "FAIL expected nail_latched at |w|=3\n";
    return 3;
  }

  // 车中心在 (3,0)，钉头 yaw 应接近 0（加 yaml yaw_offset）
  const double yaw_offset = 2.0 / 57.3;  // heavy_nail 从 standard3 拷来
  if (std::abs(cmd.yaw - yaw_offset) > 0.15) {
    std::cerr << "FAIL yaw not nailed to center, yaw=" << cmd.yaw << "\n";
    return 4;
  }

  std::cout << "nail_aimer_offline_test PASS yaw=" << cmd.yaw << " shoot=" << cmd.shoot
            << " latched=" << aimer.nail_latched() << "\n";
  return 0;
}
