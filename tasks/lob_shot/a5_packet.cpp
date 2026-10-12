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
  // 与同济 tools::get_crc16 / 本队驱动同一张表
  const uint16_t crc = tools::get_crc16(body, 11);
  std::vector<uint8_t> out(13);
  std::memcpy(out.data(), body, 11);
  out[11] = static_cast<uint8_t>(crc & 0xFF);
  out[12] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  return out;
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
