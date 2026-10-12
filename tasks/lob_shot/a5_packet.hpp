#ifndef LOB_SHOT__A5_PACKET_HPP_
#define LOB_SHOT__A5_PACKET_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace lob_shot
{

// 本队 0xA5 帧；CRC 走同济 tools::get_crc16（与 jlcv_serial 同表）
std::vector<uint8_t> encode_a5(float pitch, float yaw, uint8_t z, uint8_t deploy = 0);
std::string to_hex(const std::vector<uint8_t> & p);

}  // namespace lob_shot

#endif
