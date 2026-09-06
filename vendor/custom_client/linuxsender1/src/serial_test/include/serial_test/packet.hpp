#pragma once

#include <cstdint>
#include <vector>
#include <cstring>

namespace serial_test
{

struct FrameHeader
{
    uint8_t SOF = 0xA5;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
} __attribute__((packed));

struct RobotCustomData
{
    uint8_t data[300];
} __attribute__((packed));

struct CustomClientCommand
{
    uint8_t data[30];
} __attribute__((packed));

// 0x0310 发送包（机器人→自定义客户端）
struct SendPacket0310
{
    FrameHeader header;
    uint16_t cmd_id;
    RobotCustomData data;
    uint16_t crc16;
} __attribute__((packed));

// 0x0311 发送包（自定义客户端→机器人）
struct SendPacket0311
{
    FrameHeader header;
    uint16_t cmd_id;
    CustomClientCommand data;
    uint16_t crc16;
} __attribute__((packed));

// 接收包
struct ReceivePacket
{
    FrameHeader header;
    uint16_t cmd_id;
    uint8_t data[300];
    uint16_t crc16;
} __attribute__((packed));

template<typename T>
std::vector<uint8_t> toVector(const T& packet)
{
    std::vector<uint8_t> vec(sizeof(T));
    std::memcpy(vec.data(), &packet, sizeof(T));
    return vec;
}

template<typename T>
T fromVector(const std::vector<uint8_t>& vec)
{
    T packet;
    std::memcpy(&packet, vec.data(), sizeof(T));
    return packet;
}

} // namespace serial_test