#ifndef LOB_SHOT__COMMAND_HPP_
#define LOB_SHOT__COMMAND_HPP_

#include <cstdint>

namespace lob_shot
{

// 对齐同济 io::Command；后续可直接塞进 CBoard/Gimbal
struct Command
{
  bool control = false;
  bool shoot = false;
  double yaw = 0;    // rad
  double pitch = 0;  // rad，抬头为正
};

inline int to_z(const Command & c)
{
  if (!c.control) return 0;
  return c.shoot ? 2 : 1;
}

// 同济 VisionToGimbal.mode 同构
inline uint8_t to_gimbal_mode(const Command & c)
{
  return static_cast<uint8_t>(to_z(c));
}

}  // namespace lob_shot

#endif
