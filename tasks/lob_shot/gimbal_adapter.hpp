#ifndef LOB_SHOT__GIMBAL_ADAPTER_HPP_
#define LOB_SHOT__GIMBAL_ADAPTER_HPP_

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "command.hpp"
#include "tools/crc.hpp"

namespace lob_shot
{

// yaw 坐标系（配置 backend / yaw_frame）：
// - chassis_rel：本队 0xA5，相对底盘（Manager 输出）
// - imu_world：同济 VisionToGimbal，需在上层把相对角换成世界角后再填

enum class SendBackend
{
  DryA5,
  SerialA5,
  DryGimbalSP,  // 只组同济 SP 帧到内存，不写设备
};

struct GimbalSpFrame
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode = 0;
  float yaw = 0;
  float yaw_vel = 0;
  float yaw_acc = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float pitch_acc = 0;
  uint16_t crc16 = 0;
} __attribute__((packed));

inline GimbalSpFrame to_gimbal_sp(const Command & cmd)
{
  GimbalSpFrame f;
  f.mode = to_gimbal_mode(cmd);
  f.yaw = static_cast<float>(cmd.yaw);
  // lob Command::pitch 抬头为正；同济 SP / Planner 抬头为负
  f.pitch = static_cast<float>(-cmd.pitch);
  f.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&f), sizeof(f) - sizeof(f.crc16));
  return f;
}

inline std::vector<uint8_t> gimbal_sp_bytes(const Command & cmd)
{
  GimbalSpFrame f = to_gimbal_sp(cmd);
  std::vector<uint8_t> out(sizeof(f));
  std::memcpy(out.data(), &f, sizeof(f));
  return out;
}

inline std::string backend_name(SendBackend b)
{
  switch (b) {
    case SendBackend::DryA5: return "dry_a5";
    case SendBackend::SerialA5: return "serial_a5";
    case SendBackend::DryGimbalSP: return "dry_gimbal_sp";
  }
  return "unknown";
}

}  // namespace lob_shot

#endif
