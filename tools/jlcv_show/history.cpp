#include "dashboard.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace tools::jlcv_show {
Eigen::Vector3d plate_normal(const ArmorPose& p) {
  return {-std::cos(p.xyza[3])*std::cos(p.pitch),-std::sin(p.xyza[3])*std::cos(p.pitch),std::sin(p.pitch)};
}
Eigen::Vector3d plate_up(const ArmorPose& p) {
  return {std::cos(p.xyza[3])*std::sin(p.pitch),std::sin(p.xyza[3])*std::sin(p.pitch),std::cos(p.pitch)};
}
namespace {
constexpr double degrees=180/3.14159265358979323846;
double unwrap(double angle,const std::optional<double>& previous,const std::optional<double>& other) {
  const auto reference=previous ? previous:other;
  return reference ? *reference+std::remainder(angle-*reference,360.):angle;
}
}
Range value_range(const std::vector<Point>& a,const std::vector<Point>& b,double since,double span) {
  double lo=std::numeric_limits<double>::infinity(),hi=-lo;
  for(const auto* series:{&a,&b})for(const auto& p:*series)
    if(p.time>=since && p.valid && std::isfinite(p.value)) {
      lo=std::min(lo,p.value);hi=std::max(hi,p.value);
    }
  if(!std::isfinite(lo))return {-1,1};
  const double center=(lo+hi)/2,half=std::max(span,hi-lo)*.62;
  return {center-half,center+half};
}
void Channel::append(Curve curve,double time,double value,bool valid) {
  if(!std::isfinite(time) || time<0 || (valid && !std::isfinite(value)))return;
  auto& series=curves_[curve];
  if(!series.empty() && time<series.back().time)return;
  if(!series.empty() && time==series.back().time)series.back()={time,value,valid};
  else series.push_back({time,value,valid});
  latest_.time=std::max(latest_.time,time);
  for(auto& points:curves_) {
    while(points.size()>2 && points[1].time<latest_.time-history_seconds)points.pop_front();
    while(points.size()>maximum_points)points.pop_front();
  }
}
void Channel::reset(std::string source) {
  std::lock_guard<std::mutex> lock(mutex_);
  for(auto& c:curves_)c.clear();
  latest_=Snapshot{};latest_.source=std::move(source);
  feedback_at_=estimate_at_=preview_at_=Clock::time_point{};
  sent_yaw_.reset();returned_yaw_.reset();
}
void Channel::status(std::string message) {
  std::lock_guard<std::mutex> lock(mutex_);latest_.status=std::move(message);
}
void Channel::feedback(double time,double yaw,double pitch) {
  if(!std::isfinite(time) || time<0 || !std::isfinite(yaw) || !std::isfinite(pitch))return;
  std::lock_guard<std::mutex> lock(mutex_);
  if(!curves_[feedback_yaw].empty() && time<=curves_[feedback_yaw].back().time)return;
  const double y=unwrap(yaw*degrees,returned_yaw_,sent_yaw_);
  append(feedback_yaw,time,y);append(feedback_pitch,time,pitch*degrees);
  returned_yaw_=y;feedback_at_=Clock::now();
}
void Channel::command(double time,double yaw,double pitch,bool control,bool shoot) {
  if(!std::isfinite(time) || time<0)return;
  std::lock_guard<std::mutex> lock(mutex_);
  if(!curves_[command_yaw].empty() && time<curves_[command_yaw].back().time)return;
  const double y=unwrap(yaw*degrees,sent_yaw_,returned_yaw_);
  const bool valid=control && std::isfinite(yaw) && std::isfinite(pitch);
  append(command_yaw,time,y,valid);append(command_pitch,time,pitch*degrees,valid);
  append(command_shoot,time,shoot?1:0);
  if(valid)sent_yaw_=y;
  latest_.control=control;latest_.shoot=shoot;
}
void Channel::estimate(std::vector<ArmorPose> armors,std::optional<Eigen::Vector3d> aim,
                       std::string target,const cv::Mat& image) {
  armors.erase(std::remove_if(armors.begin(),armors.end(),[](const auto& p){return !p.xyza.allFinite();}),armors.end());
  if(aim && !aim->allFinite())aim.reset();
  const auto now=Clock::now();
  cv::Mat preview;
  // Preview is a small, bounded copy; image rendering never queues the full camera stream.
  if(!image.empty() && now-preview_at_>=std::chrono::milliseconds(70)) {
    const double scale=std::min(420./image.cols,280./image.rows);
    cv::resize(image,preview,{},scale,scale,cv::INTER_AREA);preview_at_=now;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.armors=std::move(armors);latest_.aim=std::move(aim);latest_.target=std::move(target);
  if(!preview.empty())latest_.preview=std::move(preview);
  estimate_at_=now;
}
Snapshot Channel::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);auto result=latest_;
  for(size_t i=0;i<curve_count;++i)result.curves[i].assign(curves_[i].begin(),curves_[i].end());
  const auto now=Clock::now();
  if(feedback_at_!=Clock::time_point{})result.feedback_age=std::chrono::duration<double>(now-feedback_at_).count();
  if(estimate_at_!=Clock::time_point{})result.estimate_age=std::chrono::duration<double>(now-estimate_at_).count();
  return result;
}
}
