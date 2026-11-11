#pragma once
#include "rm_sim/adapters/external.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>

namespace rm_sim::adapters {
// Protocol I/O worker: world time and UI rendering do not wait for algorithm processing.
class LiveExternal final:public Algorithm {
 public:
  LiveExternal(ExternalConfig,RunnerConfig);
  ~LiveExternal();
  std::string version() const override {return "external-tcp-live/1";}
  void start();
  std::vector<ControlRequest> update(const AlgorithmInput&) override;
  void finish(Tick);
  nlohmann::json statistics();
 private:
  ExternalConfig config_;
  RunnerConfig context_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<AlgorithmInput> pending_;
  std::vector<ControlRequest> ready_;
  std::exception_ptr error_;
  nlohmann::json statistics_;
  std::thread worker_;
  bool quit_=false,clean_finish_=false;
  Tick end_tick_=0;
  uint64_t dropped_=0;
  void work();
};
} // namespace rm_sim::adapters
