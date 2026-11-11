#pragma once
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstdint>
#include <string>
#include <stdexcept>
#include <vector>

namespace rm_sim::adapters {
inline constexpr const char* wire_version="rm-sim/1";
inline constexpr size_t max_header_bytes=1024*1024;
inline constexpr size_t max_payload_bytes=64*1024*1024;
using Deadline=std::chrono::steady_clock::time_point;
// Recoverable transport loss, distinct from invalid protocol messages.
class TransportError:public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
Deadline deadline_after(int milliseconds);
void validate_endpoint(const std::string& ipv4,int port);
void validate_telemetry(const nlohmann::json&);
struct Message {
  nlohmann::json header;
  std::vector<uint8_t> payload;
};
// Nonblocking sockets with one absolute deadline per complete exchange.
class Connection {
 public:
  Connection()=default;
  explicit Connection(int fd):fd_(fd) {}
  ~Connection();
  Connection(const Connection&)=delete;
  Connection& operator=(const Connection&)=delete;
  Connection(Connection&& other) noexcept;
  Connection& operator=(Connection&& other) noexcept;
  static Connection connect(const std::string& ipv4,int port,Deadline deadline);
  void send(const Message&,Deadline);
  Message receive(Deadline);
 private:
  int fd_=-1;
  void transfer(void* data,size_t size,bool writing,Deadline deadline);
};
class Listener {
 public:
  Listener(const std::string& ipv4,int port);
  ~Listener();
  Listener(const Listener&)=delete;
  Listener& operator=(const Listener&)=delete;
  Connection accept(Deadline);
 private:
  int fd_=-1;
};
} // namespace rm_sim::adapters
