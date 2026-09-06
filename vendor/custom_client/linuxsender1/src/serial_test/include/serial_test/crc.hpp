// crc.hpp
#pragma once

#include <cstdint>

namespace serial_test
{

namespace crc16
{

constexpr uint16_t CRC16_INIT = 0xFFFF;

extern const uint16_t W_CRC_TABLE[256];

uint16_t Get_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);

uint32_t Verify_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength);

void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);

} // namespace crc16

namespace crc8
{

constexpr uint8_t CRC8_INIT = 0xFF;

extern const uint8_t CRC8_TAB[256];

uint8_t Get_CRC8_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength, uint8_t ucCRC8);

uint32_t Verify_CRC8_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength);

void Append_CRC8_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);

} // namespace crc8

} // namespace serial_test
