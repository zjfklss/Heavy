#include "a5_packet.hpp"

#include <cstring>

#include "tools/crc.hpp"

namespace lob_shot
{

std::vector<uint8_t> encode_a5(float pitch, float yaw, uint8_t z, uint8_t deploy)
{
  uint8_t body[11];
  body[0] = 0xA5;
  std::memcpy(body + 1, &pitch, 4);
  std::memcpy(body + 5, &yaw, 4);
  body[9] = z;
  body[10] = deploy;
  // 与英雄 jlcv_serial_driver / 同济 tools::get_crc16 同一张表
  const uint16_t crc = tools::get_crc16(body, 11);
  std::vector<uint8_t> out(13);
  std::memcpy(out.data(), body, 11);
  out[11] = static_cast<uint8_t>(crc & 0xFF);
  out[12] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  return out;
}

bool decode_5a(const uint8_t * data, std::size_t len, ReceivePacket5A & out)
{
  if (!data || len < sizeof(ReceivePacket5A)) return false;
  if (data[0] != 0x5A) return false;
  if (!tools::check_crc16(data, sizeof(ReceivePacket5A))) return false;
  std::memcpy(&out, data, sizeof(ReceivePacket5A));
  return true;
}

std::string to_hex(const std::vector<uint8_t> & p)
{
  static const char * kHex = "0123456789abcdef";
  std::string s;
  s.reserve(p.size() * 2);
  for (uint8_t b : p) {
    s.push_back(kHex[b >> 4]);
    s.push_back(kHex[b & 0xF]);
  }
  return s;
}

}  // namespace lob_shot
