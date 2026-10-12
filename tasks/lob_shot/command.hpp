#ifndef LOB_SHOT__COMMAND_HPP_
#define LOB_SHOT__COMMAND_HPP_

#include <cstdint>

namespace lob_shot
{

// 对齐英雄 0xA5 语义：z=0 空闲 / 1 瞄准 / 2 开火；deploy 对应 deploy_mode_enabled
struct Command
{
  bool control = false;
  bool shoot = false;
  bool deploy = false;  // 英雄串口 deploy_mode_enabled
  double yaw = 0;       // rad
  double pitch = 0;     // rad，抬头为正
};

inline int to_z(const Command & c)
{
  if (!c.control) return 0;
  return c.shoot ? 2 : 1;
}

inline uint8_t to_deploy(const Command & c) { return c.deploy ? 1 : 0; }

// 同济 VisionToGimbal.mode 同构
inline uint8_t to_gimbal_mode(const Command & c)
{
  return static_cast<uint8_t>(to_z(c));
}

}  // namespace lob_shot

#endif
