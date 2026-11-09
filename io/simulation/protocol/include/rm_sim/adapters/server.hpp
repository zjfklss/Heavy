#pragma once
#include "rm_sim/adapters/wire.hpp"
#include <memory>
#include <optional>

namespace rm_sim::adapters {
struct ServerOptions {
  std::string host="127.0.0.1",adapter="generic",peer_version="external-algorithm/1";
  int port=17001,accept_timeout_ms=0,idle_timeout_ms=10000; // 0 accepts indefinitely
};
struct Command {
  bool control=false;
  double yaw=0,pitch=0; // external algorithm's convention, as negotiated in hello
  bool fire=false;
  std::string request_id;
};
// Algorithm-side protocol SDK. Contains no detector, tracker, predictor or aiming logic.
// Images are raw BGR8 spans in Message::payload; metadata offsets are in header["images"].
class ProtocolServer {
 public:
  explicit ProtocolServer(ServerOptions options={});
  void accept();
  const nlohmann::json& hello() const { return hello_; }
  std::optional<Message> next(); // nullopt after acknowledging a clean finish
  void reply(const std::vector<Command>& commands,
             const nlohmann::json& telemetry=nlohmann::json::object());
 private:
  ServerOptions options_;
  std::unique_ptr<Listener> listener_;
  Connection connection_;
  nlohmann::json hello_,pending_;
  uint64_t sequence_=0,last_tick_=0;
  bool active_=false;
};
} // namespace rm_sim::adapters
