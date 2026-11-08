// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace jlcv_serial_driver
{
enum
{
  // 0x5A
  RECEVIDE_HEADER = 0x5A,
  // 0xA5 
  SEND_HEADER_ANGLE = 0xA5,
};

struct ReceivePacket
{
  uint8_t header = RECEVIDE_HEADER;
  uint8_t enemy_color : 1;  // 敌方颜色  0-red 1-blue
  uint8_t game_status : 3;  // 比赛状态
  uint8_t outpost_status : 1;  // enemy前哨站状态
  uint8_t reset_tracker : 1;   // 重置跟踪器

  uint8_t camera_status : 1;  // 相机状态 

  float curr_pitch;
  float curr_yaw;
  float bullet_speed;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacketAngle
{
  // geometry_msgs/msg/Vector3
  float x; //pitch
  float y; //yaw
  uint8_t z; //control_status 0/1/2 0表示没识别到
  uint8_t deploy_mode_enabled; //0非部署模式 1部署模式
} __attribute__((packed));

struct SendPacket
{
  // 0xA5: armor
  uint8_t header;
  union Data
  { 
    struct SendPacketAngle packetangle;
  } data;
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t>& data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t*>(&packet));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T& data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(reinterpret_cast<const uint8_t*>(&data), reinterpret_cast<const uint8_t*>(&data) + sizeof(T),
            packet.begin());
  return packet;
}

}  // namespace jlcv_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_