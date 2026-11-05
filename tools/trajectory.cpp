#include "trajectory.hpp"

#include <array>
#include <cmath>

namespace tools
{
namespace
{
using State = std::array<double, 4>;
State derivative(const State & state, double drag)
{
  const double speed = std::hypot(state[2], state[3]);
  return {state[2], state[3], -drag * speed * state[2],
          -9.7915 - drag * speed * state[3]};
}
State advance(const State & state, const State & change, double step)
{
  State result;
  for (size_t index = 0; index < result.size(); ++index)
    result[index] = state[index] + step * change[index];
  return result;
}
bool simulate(double speed, double distance, double angle, double drag,
              double & height, double & time)
{
  State state{0, 0, speed * std::cos(angle), speed * std::sin(angle)};
  constexpr double step = 0.001;
  for (int iteration = 0; iteration < 5000; ++iteration) {
    const auto first = derivative(state, drag);
    const auto second = derivative(advance(state, first, step / 2), drag);
    const auto third = derivative(advance(state, second, step / 2), drag);
    const auto fourth = derivative(advance(state, third, step), drag);
    auto next = state;
    for (size_t index = 0; index < state.size(); ++index)
      next[index] += step * (first[index] + 2 * second[index] + 2 * third[index] + fourth[index]) / 6;
    if (next[0] >= distance) {
      const double fraction = (distance - state[0]) / (next[0] - state[0]);
      height = state[1] + fraction * (next[1] - state[1]);
      time = (iteration + fraction) * step;
      return std::isfinite(height) && std::isfinite(time);
    }
    if (next[2] <= 0 || !std::isfinite(next[1])) return false;
    state = next;
  }
  return false;
}
}

Trajectory::Trajectory(double speed, double distance, double height, double drag)
{
  if (!std::isfinite(speed) || !std::isfinite(distance) || !std::isfinite(height) ||
      !std::isfinite(drag) || speed <= 0 || distance <= 0 || drag < 0) return;
  double lower = std::atan2(height, distance);
  double lower_height = 0, time = 0;
  if (!simulate(speed, distance, lower, drag, lower_height, time)) return;
  double upper = lower;
  double upper_height = lower_height;
  while (upper < 1.5 && upper_height < height) {
    upper += 0.025;
    if (!simulate(speed, distance, upper, drag, upper_height, time)) return;
  }
  if (upper_height < height) return;
  for (int iteration = 0; iteration < 36; ++iteration) {
    const double middle = (lower + upper) / 2;
    double predicted_height = 0;
    if (!simulate(speed, distance, middle, drag, predicted_height, time)) return;
    if (predicted_height < height) lower = middle;
    else upper = middle;
  }
  pitch = (lower + upper) / 2;
  if (!simulate(speed, distance, pitch, drag, lower_height, fly_time)) return;
  unsolvable = std::abs(lower_height - height) > 0.001;
}
}
