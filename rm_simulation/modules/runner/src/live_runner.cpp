#include "rm_sim/runner.hpp"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>

namespace rm_sim {
namespace {
// Rendering never holds the world clock. One pending capture and one completed frame bound memory.
class LiveCamera {
 public:
  struct Job {Snapshot world;Pose pose;Tick tick;};
  explicit LiveCamera(Sensor& sensor):sensor_(sensor),worker_([this]{work();}){}
  ~LiveCamera(){
    {std::lock_guard<std::mutex> lock(mutex_);quit_=true;condition_.notify_one();}
    worker_.join();
  }
  void submit(Job job){
    std::lock_guard<std::mutex> lock(mutex_);
    if(pending_)++dropped_;
    pending_=std::move(job);condition_.notify_one();
  }
  std::optional<SensorSample> take(){
    std::lock_guard<std::mutex> lock(mutex_);
    if(error_)std::rethrow_exception(error_);
    auto result=std::move(completed_);completed_.reset();return result;
  }
  uint64_t dropped(){std::lock_guard<std::mutex> lock(mutex_);return dropped_;}
 private:
  Sensor& sensor_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool quit_=false;
  uint64_t dropped_=0;
  std::optional<Job> pending_;
  std::optional<SensorSample> completed_;
  std::exception_ptr error_;
  std::thread worker_;
  void work(){
    try{
      for(;;){
        Job job;
        {std::unique_lock<std::mutex> lock(mutex_);condition_.wait(lock,[&]{return quit_ || pending_.has_value();});
          if(quit_)return;
          job=std::move(*pending_);pending_.reset();}
        auto sample=sensor_.capture(job.world,job.pose,job.tick);
        {std::lock_guard<std::mutex> lock(mutex_);
          if(completed_)++dropped_;
          completed_=std::move(sample);}
      }
    }catch(...){std::lock_guard<std::mutex> lock(mutex_);error_=std::current_exception();}
  }
};
}
Tick Runner::run_live(Algorithm& algorithm,const std::function<bool()>& stop,
    const std::function<void(const LiveUpdate&)>& publish,double time_limit){
  ensure(!started_,"Runner is single-use");
  ensure(std::isfinite(time_limit) && time_limit>=0,"Invalid live time limit");
  // Use the maximum scheduling horizon to validate queues, not as an automatic stop time.
  validate_duration(static_cast<Tick>(std::min(10000000.,600/config_.world.timestep)));
  started_=true;recording_=false;log_=RunLog{};
  LiveCamera camera(sensor_);
  LiveUpdate update;
  AlgorithmInput inbox;
  PeriodicClock display(30,config_.world.timestep);
  size_t event_index=0,shot_index=0,hit_index=0;
  using Clock=std::chrono::steady_clock;
  const auto begin=Clock::now();
  auto seconds=[&]{return std::chrono::duration<double>(Clock::now()-begin).count();};
  auto publish_now=[&]{
    update.frame={world_.snapshot(),actuator_.truth()};update.wall_time=seconds();
    update.commands=live_commands_;update.dropped_camera_jobs=camera.dropped();
    const auto& hits=world_.hits();
    update.contacts=hits.size();
    for(;hit_index<hits.size();++hit_index)update.effective_hits+=hits[hit_index].effective;
    // Keep every impact that can still have a visible five-second plate mark,
    // as well as at least the last 128 events for the monitoring list.
    auto recent=std::lower_bound(hits.begin(),hits.end(),world_.time()-5.,
      [](const Hit& hit,double time){return hit.time<time;});
    auto history=hits.begin()+(hits.size()>128 ? hits.size()-128:0);
    update.hits.assign(std::min(recent,history),hits.end());
    publish(update);
  };
  while(!stop()){
    double elapsed=seconds();
    if(time_limit>0 && tick_*config_.world.timestep>=time_limit)break;
    if(tick_*config_.world.timestep>elapsed){
      std::this_thread::sleep_until(begin+std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(tick_*config_.world.timestep)));
      continue;
    }
    while(event_index<events_.size() && events_[event_index].tick<=tick_){
      const auto& event=events_[event_index++];
      if(event.type=="stop_outpost")world_.stop_outpost(event.target_id);
      else world_.activate_rune(event.target_id);
    }
    while(!commands_.empty() && commands_.begin()->first.first<=tick_){
      actuator_.apply(commands_.begin()->second);commands_.erase(commands_.begin());
    }
    while(shot_index<shots_.size() && shots_[shot_index].tick<=tick_){
      world_.launch(shots_[shot_index++].shot);++update.shots;
    }
    for(const auto& event:actuator_.launches()){world_.launch(event.shot);++update.shots;}
    // Never render old camera deadlines while catching up to the wall clock.
    if(config_.sensor.enabled && sensor_clock_.take(tick_) &&
        elapsed-tick_*config_.world.timestep<.005)
      camera.submit({world_.snapshot(),actuator_.truth().camera_pose,tick_});
    if(auto sample=camera.take()){
      auto& packet=sample->packet;
      packet.ready_tick=std::max(packet.ready_tick,tick_);
      packet.ready_time=packet.ready_tick*config_.world.timestep;
      observations_.push_back(std::move(packet));++update.camera_frames;
    }
    if(config_.actuator.feedback_enabled && feedback_clock_.take(tick_))feedback_.push_back(actuator_.feedback());
    while(!observations_.empty() && observations_.front().ready_tick<=tick_){
      if(observations_.front().image)update.camera=observations_.front();
      inbox.observations.push_back(std::move(observations_.front()));observations_.pop_front();
    }
    while(!feedback_.empty() && feedback_.front().ready_tick<=tick_){
      inbox.feedback.push_back(feedback_.front());feedback_.pop_front();
    }
    if(algorithm_clock_.take(tick_)){
      inbox.tick=tick_;inbox.time=tick_*config_.world.timestep;inbox.timestep=config_.world.timestep;
      for(auto request:algorithm.update(inbox))
        enqueue(std::move(request),tick_+std::max<Tick>(1,quantize_delay(config_.actuator.command_latency,config_.world.timestep)));
      inbox.observations.clear();inbox.feedback.clear();
    }
    if(display.take(tick_))publish_now();
    world_.step();actuator_.advance();++tick_;
  }
  publish_now();return tick_;
}
} // namespace rm_sim
