#include "lob_serial.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <iostream>

namespace lob_shot
{

void DrySender::send(const Command & cmd)
{
  const int z = to_z(cmd);
  last_a5_ = encode_a5(
    static_cast<float>(cmd.pitch), static_cast<float>(cmd.yaw), static_cast<uint8_t>(z),
    to_deploy(cmd));
  last_ = "dry z=" + std::to_string(z) + " deploy=" + std::to_string(to_deploy(cmd)) +
          " a5=" + to_hex(last_a5_);
  std::cout << last_ << "\n";
}

Feedback DrySender::read_feedback() { return {}; }

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
  else if (baud_ == 460800) sp = B460800;
  cfsetispeed(&tty, sp);
  cfsetospeed(&tty, sp);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  rx_.clear();
  return true;
}

void SerialSender::send(const Command & cmd)
{
  if (fd_ < 0) return;
  const int z = to_z(cmd);
  auto pkt = encode_a5(
    static_cast<float>(cmd.pitch), static_cast<float>(cmd.yaw), static_cast<uint8_t>(z),
    to_deploy(cmd));
  const ssize_t n = ::write(fd_, pkt.data(), pkt.size());
  if (n < 0) {
    // 保持与英雄 reopen 语义一致：调用方看日志；此处不抛
    perror("SerialSender write");
  }
}

bool SerialSender::feed_byte(uint8_t b, Feedback & out)
{
  if (rx_.empty()) {
    if (b != 0x5A) return false;
    rx_.push_back(b);
    return false;
  }
  rx_.push_back(b);
  if (rx_.size() < sizeof(ReceivePacket5A)) return false;

  ReceivePacket5A pkt{};
  if (!decode_5a(rx_.data(), rx_.size(), pkt)) {
    // 失步：丢掉首字节，从缓冲里重新找头
    rx_.erase(rx_.begin());
    return false;
  }
  out.ok = true;
  out.pitch = pkt.curr_pitch;
  out.yaw = pkt.curr_yaw;
  out.bullet_speed = pkt.bullet_speed;
  rx_.clear();
  return true;
}

Feedback SerialSender::read_feedback()
{
  Feedback out;
  if (fd_ < 0) return out;
  uint8_t buf[256];
  const ssize_t n = ::read(fd_, buf, sizeof(buf));
  if (n <= 0) return out;
  for (ssize_t i = 0; i < n; ++i) {
    if (feed_byte(buf[i], out)) return out;
  }
  return out;
}

}  // namespace lob_shot
