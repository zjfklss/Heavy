#ifndef AUTO_AIM__HERO_SHOOTER_HPP
#define AUTO_AIM__HERO_SHOOTER_HPP

#include <chrono>
#include <string>
#include <vector>

#include "io/command.hpp"
#include <thread>
#include "tasks/auto_aim/target.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_outpost
{
int select_layer(const std::vector<Eigen::Vector4d> & armors, double current_pitch, double threshold);
// {valid,y,p,id,shoot_time}
struct shooter_info
{
  bool valid;
  double yaw;
  double pitch;
  int aim_id;
  std::chrono::steady_clock::time_point shoot_time;
  shooter_info() : valid(false), yaw(0.0), pitch(0.0), aim_id(0), shoot_time(std::chrono::steady_clock::now()) {}
  shooter_info(
    bool _valid, double _yaw, double _pitch, int _aim_id,
    std::chrono::steady_clock::time_point _shoot_time = std::chrono::steady_clock::time_point{})
  : valid(_valid), yaw(_yaw), pitch(_pitch), aim_id(_aim_id), shoot_time(_shoot_time)
  {
  }
};
// {valid，xyza}
struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Shooter
{
public:
  explicit Shooter(const std::string & config_path);
  void schedule(auto_aim::Target target_at_t0);
  int get_next_armor(const auto_aim::Target & target, double flytime, std::chrono::steady_clock::time_point timestamp);
  io::Command shoot(
    std::list<auto_aim::Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed, bool to_now, Eigen::Vector3d ypr);
  Eigen::Vector3d get_outpost_front(const auto_aim::Target & target, double current_pitch = 0);
  Eigen::Vector3d get_top_front(const auto_aim::Target & target);

  AimPoint choose_aim_point(const auto_aim::Target & target, double current_pitch = 0);

  ~Shooter();
  AimPoint debug_aim_point_;

private:
  // 装甲板的三个状态：允许击打，禁止击打，等待。
  // 每次击打允许击打的装甲板后，该装甲板变为禁止，下一个变为等待，其余变为允许。想要击打等待状态的装甲板，拒绝击打，该装甲板变为禁止，其余变为允许。
  enum State { ALLOW = 0b00, SHOOTED = 0b01, SKIP = 0b10 };
// 设置状态的宏
#define SET_STATE(bitmap, object, state) \
  (bitmap = (bitmap & ~(0b11 << (object * 2))) | (static_cast<uint8_t>(state) << (object * 2)))

// 获取状态的宏
#define GET_STATE(bitmap, object) static_cast<State>((bitmap >> (object * 2)) & 0b11)

  double ctrl_to_fire_;  // 自瞄控制，发射命令至摩擦轮减速的延迟
  double mouse_to_fire_; // 鼠标操控，发射命令至摩擦轮减速的延迟
  double yaw_offset_;
  double pitch_offset_;
  double direction_;
  double drag_;
  double layer_threshold_;
  double max_fire_error_yaw_;
  double max_fire_error_pitch_;
  double max_shoot_angle_;
  int last_hit_id_ = -1;
  int lock_id_ = -1;
  bool exit_;
  std::thread when_to_fire_;
  tools::ThreadSafeQueue<shooter_info> queue_0_, queue_1_, queue_2_;
  int choose_armor(std::vector<std::chrono::steady_clock::time_point> send_times, std::chrono::steady_clock::time_point now)
  {
    auto erliest_send_time = now + std::chrono::seconds(10);
    int id = 0;
    for (int i = 0; i < send_times.size(); i++) {
      if (send_times[i] > now - std::chrono::milliseconds(30) && send_times[i] < erliest_send_time) {
        erliest_send_time = send_times[i];
        id = i;
      }
    }
    return id;
  }
};
}  // namespace auto_outpost
#endif
