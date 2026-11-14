#pragma once

namespace tools
{
// 根据弹速、水平距离、高差和空气阻力系数求解低抛弹道仰角与飞行时间；
// 单位：speed(m/s)、distance(m)、height(m)、fly_time(s)、pitch(rad)。无可行解时 unsolvable 为 true。
struct Trajectory
{
  bool unsolvable = true;
  double fly_time = 0;
  double pitch = 0;
  Trajectory(double speed, double distance, double height, double drag = 0.0068);
};
}
