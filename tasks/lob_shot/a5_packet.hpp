#ifndef LOB_SHOT__A5_PACKET_HPP_
#define LOB_SHOT__A5_PACKET_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace lob_shot
{

// 对齐本队英雄 jlcv_serial_driver/packet.hpp（远程 jlauto-aim/hero）
// 发送 0xA5：pitch/yaw/z/deploy + CRC16
// 接收 0x5A：姿态 + 弹速 + 状态位 + CRC16

#pragma pack(push, 1)
struct ReceivePacket5A
{
  uint8_t header = 0x5A;
  uint8_t enemy_color : 1;
  uint8_t game_status : 3;
  uint8_t outpost_status : 1;
  uint8_t reset_tracker : 1;
  uint8_t camera_status : 1;
  uint8_t reserved : 1;
  float curr_pitch = 0;
  float curr_yaw = 0;
  float bullet_speed = 0;
  uint16_t checksum = 0;
};
#pragma pack(pop)

static_assert(sizeof(ReceivePacket5A) == 16, "ReceivePacket5A must match hero 0x5A layout");

std::vector<uint8_t> encode_a5(float pitch, float yaw, uint8_t z, uint8_t deploy = 0);

// 从完整 16 字节缓冲解析；CRC 失败返回 false
bool decode_5a(const uint8_t * data, std::size_t len, ReceivePacket5A & out);

std::string to_hex(const std::vector<uint8_t> & p);

}  // namespace lob_shot

#endif
