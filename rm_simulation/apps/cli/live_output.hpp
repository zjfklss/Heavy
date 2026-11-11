#pragma once
#include "rm_sim/image.hpp"
#include <nlohmann/json.hpp>
#include <arpa/inet.h>
#include <condition_variable>
#include <cstdio>
#include <exception>
#include <mutex>
#include <optional>
#include <thread>

// Private UI display transport. Evaluation world frames never enter the algorithm socket.
class LiveOutput {
 public:
  LiveOutput():worker_([this]{work();}){}
  ~LiveOutput(){close();}
  void publish(nlohmann::json header,std::shared_ptr<const rm_sim::Image> image){
    std::lock_guard<std::mutex> lock(mutex_);
    if(error_)std::rethrow_exception(error_);
    pending_=Frame{std::move(header),std::move(image)};condition_.notify_one();
  }
  void close(){
    {std::lock_guard<std::mutex> lock(mutex_);quit_=true;condition_.notify_one();}
    if(worker_.joinable())worker_.join();
  }
 private:
  struct Frame {nlohmann::json header;std::shared_ptr<const rm_sim::Image> image;};
  std::mutex mutex_;std::condition_variable condition_;
  bool quit_=false;std::optional<Frame> pending_;std::exception_ptr error_;std::thread worker_;
  static void write(const void* data,size_t bytes){
    if(bytes && std::fwrite(data,1,bytes,stdout)!=bytes)throw std::runtime_error("UI live stream closed");
  }
  void work(){
    try{
      for(;;){
        Frame frame;
        {std::unique_lock<std::mutex> lock(mutex_);condition_.wait(lock,[&]{return quit_ || pending_.has_value();});
          if(!pending_ && quit_)return;
          frame=std::move(*pending_);pending_.reset();}
        auto header=frame.header.dump();size_t bytes=frame.image?frame.image->bgr.size():0;
        rm_sim::ensure(header.size()<=1024*1024 && bytes<=64*1024*1024,"Live frame exceeds display limit");
        uint32_t prefix[]{htonl(0x524d4c31),htonl(uint32_t(header.size())),htonl(uint32_t(bytes))};
        write(prefix,sizeof(prefix));write(header.data(),header.size());
        if(frame.image)write(frame.image->bgr.data(),bytes);
        if(std::fflush(stdout))throw std::runtime_error("Cannot flush UI live stream");
      }
    }catch(...){std::lock_guard<std::mutex> lock(mutex_);error_=std::current_exception();}
  }
};
