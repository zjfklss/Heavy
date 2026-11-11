#pragma once
#include "rm_sim/runner.hpp"
#include "rm_sim/adapters/profile.hpp"
#include "rm_sim/adapters/wire.hpp"

namespace rm_sim::adapters {
struct ExternalConfig {
  std::string adapter="generic",host="127.0.0.1";
  int port=17001;
  double connect_timeout=5,step_timeout=1; // wall-clock seconds, independent of simulation time
  void validate(const RunnerConfig&) const;
};
ExternalConfig parse_config(const nlohmann::json& algorithm);
nlohmann::json encode_config(const ExternalConfig&);
class ExternalAlgorithm final:public Algorithm {
 public:
  ExternalAlgorithm(ExternalConfig,RunnerConfig,Tick duration,bool realtime=false);
  std::string version() const override;
  void start(); // No I/O during construction/configuration validation.
  std::vector<ControlRequest> update(const AlgorithmInput&) override;
  void finish(Tick end_tick);
  nlohmann::json statistics() const;
 private:
  ExternalConfig config_;
  RunnerConfig context_;
  Tick duration_;
  Connection connection_;
  std::string session_,peer_version_;
  uint64_t sequence_=0,images_=0,feedback_=0,commands_=0;
  bool started_=false,finished_=false;
  bool realtime_=false;
  nlohmann::json telemetry_=nlohmann::json::object();
};
} // namespace rm_sim::adapters
