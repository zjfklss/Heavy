#include "lob_serial.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <iostream>

#include "a5_packet.hpp"

namespace lob_shot
{

void DrySender::send(const Command & cmd)
{
  const int z = to_z(cmd);
  last_a5_ = encode_a5(
    static_cast<float>(cmd.pitch), static_cast<float>(cmd.yaw), static_cast<uint8_t>(z));
  last_ = "dry z=" + std::to_string(z) + " a5=" + to_hex(last_a5_);
  std::cout << last_ << "\n";
}

bool DrySender::read_feedback(double &, double &) { return false; }

SerialSender::SerialSender(std::string port, int baud) : port_(std::move(port)), baud_(baud) {}

SerialSender::~SerialSender()
{
  if (fd_ >= 0) ::close(fd_);
}

bool SerialSender::open()
{
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  cfmakeraw(&tty);
  speed_t sp = B921600;
  if (baud_ == 115200) sp = B115200;
  cfsetispeed(&tty, sp);
  cfsetospeed(&tty, sp);
  tty.c_cflag |= (CLOCAL | CREAD);
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  return true;
}

void SerialSender::send(const Command & cmd)
{
  if (fd_ < 0) return;
  const int z = to_z(cmd);
  auto pkt = encode_a5(
    static_cast<float>(cmd.pitch), static_cast<float>(cmd.yaw), static_cast<uint8_t>(z));
  (void)::write(fd_, pkt.data(), pkt.size());
}

bool SerialSender::read_feedback(double & yaw, double & pitch)
{
  // 0x5A 回传解析：本轮若无硬件再按电控协议填满；保留调用点
  (void)yaw;
  (void)pitch;
  if (fd_ < 0) return false;
  return false;
}

}  // namespace lob_shot
