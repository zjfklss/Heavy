#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools
{
// 无空气阻力抛物线：给初速、水平距离、高度差，取低抛解。pitch 抬头为正（Aimer 下发前取负）。
struct Trajectory
{
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为正

  // 不考虑空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  Trajectory(const double v0, const double d, const double h);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP