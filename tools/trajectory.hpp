#pragma once

namespace tools
{
struct Trajectory
{
  bool unsolvable = true;
  double fly_time = 0;
  double pitch = 0;
  Trajectory(double speed, double distance, double height, double drag = 0.0068);
};
}
