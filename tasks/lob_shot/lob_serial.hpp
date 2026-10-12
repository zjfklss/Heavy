#ifndef LOB_SHOT__LOB_SERIAL_HPP_
#define LOB_SHOT__LOB_SERIAL_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "command.hpp"

namespace lob_shot
{

// 发送后端：dry-run / 真实串口（真实路径需链接时打开设备）
class ISender
{
public:
  virtual ~ISender() = default;
  virtual void send(const Command & cmd) = 0;
  virtual bool read_feedback(double & yaw, double & pitch) = 0;  // 0x5A 回传；dry-run 返回 false
};

class DrySender : public ISender
{
public:
  void send(const Command & cmd) override;
  bool read_feedback(double & yaw, double & pitch) override;
  const std::string & last_line() const { return last_; }
  const std::vector<uint8_t> & last_a5() const { return last_a5_; }

private:
  std::string last_;
  std::vector<uint8_t> last_a5_;
};

// 真实串口路径骨架：未 open 时 send 抛错或 no-op 由调用方 --dry-run 避开
class SerialSender : public ISender
{
public:
  explicit SerialSender(std::string port, int baud = 921600);
  ~SerialSender() override;
  bool open();
  void send(const Command & cmd) override;
  bool read_feedback(double & yaw, double & pitch) override;
  bool is_open() const { return fd_ >= 0; }

private:
  std::string port_;
  int baud_;
  int fd_ = -1;
};

}  // namespace lob_shot

#endif
