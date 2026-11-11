#include "rm_sim/adapters/live.hpp"
#include <memory>

namespace rm_sim::adapters {
LiveExternal::LiveExternal(ExternalConfig config,RunnerConfig context)
 :config_(std::move(config)),context_(std::move(context)){config_.validate(context_);}
LiveExternal::~LiveExternal(){
  {std::lock_guard<std::mutex> lock(mutex_);quit_=true;condition_.notify_one();}
  if(worker_.joinable())worker_.join();
}
void LiveExternal::start(){
  ensure(!worker_.joinable(),"Live protocol session is already started");
  statistics_={{"profile",config_.adapter},{"state","waiting"},{"telemetry",nlohmann::json::object()}};
  worker_=std::thread([this]{work();});
}
std::vector<ControlRequest> LiveExternal::update(const AlgorithmInput& input){
  std::lock_guard<std::mutex> lock(mutex_);
  if(error_)std::rethrow_exception(error_);
  ensure(!quit_,"Live protocol session is stopped");
  if(!pending_)pending_=AlgorithmInput{};
  auto& pending=*pending_;
  pending.tick=input.tick;pending.time=input.time;pending.timestep=input.timestep;
  // When an algorithm falls behind, send the latest image and timestamped feedback history.
  for(const auto& packet:input.observations){
    if(!pending.observations.empty()){pending.observations.clear();++dropped_;}
    pending.observations.push_back(packet);
  }
  pending.feedback.insert(pending.feedback.end(),input.feedback.begin(),input.feedback.end());
  if(pending.feedback.size()>1000)pending.feedback.erase(pending.feedback.begin(),pending.feedback.end()-1000);
  condition_.notify_one();
  auto commands=std::move(ready_);ready_.clear();return commands;
}
nlohmann::json LiveExternal::statistics(){
  std::lock_guard<std::mutex> lock(mutex_);
  auto result=statistics_;result["dropped_input_images"]=dropped_;return result;
}
void LiveExternal::finish(Tick tick){
  {std::lock_guard<std::mutex> lock(mutex_);quit_=true;clean_finish_=true;end_tick_=tick;condition_.notify_one();}
  if(worker_.joinable())worker_.join();
  if(error_)std::rethrow_exception(error_);
}
void LiveExternal::work(){
  try{
    std::unique_ptr<ExternalAlgorithm> external;
    uint64_t connections=0;
    for(;;){
      {std::lock_guard<std::mutex> lock(mutex_);if(quit_)break;}
      try{
        if(!external){
          external=std::make_unique<ExternalAlgorithm>(config_,context_,0,true);
          external->start();++connections;
          std::lock_guard<std::mutex> lock(mutex_);
          pending_.reset(); // A new peer starts with current samples, never the old backlog.
          statistics_=external->statistics();statistics_["connections"]=connections;
        }
        AlgorithmInput input;
        {std::unique_lock<std::mutex> lock(mutex_);condition_.wait(lock,[&]{return quit_ || pending_.has_value();});
          if(quit_)break;
          input=std::move(*pending_);pending_.reset();}
        auto commands=external->update(input);
        // A restarted process may reuse its shot IDs; deduplication is per protocol session.
        for(auto& command:commands)if(command.fire)
          command.request_id="session-"+std::to_string(connections)+":"+command.request_id;
        auto stats=external->statistics();stats["connections"]=connections;
        {std::lock_guard<std::mutex> lock(mutex_);
          ready_.insert(ready_.end(),commands.begin(),commands.end());
          if(ready_.size()>128)ready_.erase(ready_.begin(),ready_.end()-128);
          statistics_=std::move(stats);}
      }catch(const TransportError&){
        external.reset(); // Close the failed socket before attempting a fresh handshake.
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.clear();pending_.reset();
        statistics_={{"profile",config_.adapter},{"state","waiting"},{"connections",connections},
          {"telemetry",nlohmann::json::object()}};
        if(condition_.wait_for(lock,std::chrono::milliseconds(500),[&]{return quit_;}))break;
      }
    }
    if(clean_finish_ && external){
      try{external->finish(end_tick_);}
      catch(const TransportError&){external.reset();} // The user may have stopped the peer first.
    }
    {std::lock_guard<std::mutex> lock(mutex_);
      if(external)statistics_=external->statistics();
      else statistics_["state"]="disconnected";
      statistics_["connections"]=connections;}
  }catch(...){std::lock_guard<std::mutex> lock(mutex_);error_=std::current_exception();}
}
} // namespace rm_sim::adapters
