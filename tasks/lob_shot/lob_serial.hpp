#ifndef LOB_SHOT__LOB_SERIAL_HPP_
#define LOB_SHOT__LOB_SERIAL_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "a5_packet.hpp"
#include "command.hpp"

namespace lob_shot
{

// 英雄电控回传（0x5A）。curr_yaw / curr_pitch 与 joint_states 相同，单位弧度。
struct Feedback
{
  double yaw = 0;
  double pitch = 0;
  double bullet_speed = 0;
  bool ok = false;
  int enemy_color = 0;    // 0 红 1 蓝，与 jlcv_serial_driver 一致
  int game_status = 0;
  bool outpost = false;
  bool reset_tracker = false;
  int camera_bit = 0;     // 包内原始位；驱动里短焦/长焦是 1-bit
  std::chrono::steady_clock::time_point stamp{};
};

class ISender
{
public:
  virtual ~ISender() = default;
  virtual void send(const Command & cmd) = 0;
  virtual Feedback read_feedback() = 0;
};

class DrySender : public ISender
{
public:
  void send(const Command & cmd) override;
  Feedback read_feedback() override;
  const std::string & last_line() const { return last_; }
  const std::vector<uint8_t> & last_a5() const { return last_a5_; }

private:
  std::string last_;
  std::vector<uint8_t> last_a5_;
};

// 英雄串口：默认 /dev/ttyUSB0 @ 921600（serial_driver.yaml）
class SerialSender : public ISender
{
public:
  explicit SerialSender(std::string port = "/dev/ttyUSB0", int baud = 921600);
  ~SerialSender() override;
  bool open();
  void send(const Command & cmd) override;
  Feedback read_feedback() override;
  bool is_open() const { return fd_ >= 0; }

private:
  bool feed_byte(uint8_t b, Feedback & out);

  std::string port_;
  int baud_;
  int fd_ = -1;
  std::vector<uint8_t> rx_;
};

}  // namespace lob_shot

#endif
