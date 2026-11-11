#ifndef AUTO_AIM__HERO_SHOOTER_HPP
#define AUTO_AIM__HERO_SHOOTER_HPP

#include <chrono>
#include <string>
#include <vector>

#include "io/command.hpp"
#include <thread>
#include "tasks/auto_aim/target.hpp"
#include "tools/thread_safe_queue.hpp"

/*
 * Shooter：把"目标状态"变成"云台指令(yaw/pitch/是否开火)"。
 *
 * 数据流：Tracker(滤波后的 Target) -> Shooter::shoot() -> io::Command -> 串口 -> 电控。
 * 英雄的两种打法：
 *   1) 目标在旋转(小陀螺/前哨站)：定头。云台固定瞄旋转中心正前方，等对应装甲板转到枪口前再开火。
 *   2) 目标平动：直接瞄装甲板，当前版本不自动开火(is_fire 恒为 false，由操作手手动开火)。
 * 前哨站一律走"定头"，细节见 shoot() 与 get_outpost_front() 的注释。
 */
namespace auto_outpost
{
// 在前哨站的三块装甲板里选要打的那一层，返回下标，没有有效层返回 -1。
// armors: 每个元素 {x, y, z, yaw}；z 无效(NaN)或 0 的视为未知层，不参与选择。
// threshold: 最高两层高度差小于它时，改为选"离当前 pitch 最近"的层。
//   注意：前哨站调用时传 0 来禁用这个就近选择——它会形成正反馈(云台压低后更倾向选低层)，导致云台在两层间抖动。
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

// 一个 Shooter 对应一次"连续瞄准过程"：它保存前哨站锁定点、上次开火时间等跨帧状态。
// 串口反馈的 epoch 变化(重连/换色/重置)时由 hero.cpp 重新构造，所有状态随之清空。
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
  // 注意：shoot() 里操作的是 Target 的局部拷贝，状态位没有写回 Target，所以这套状态机目前实际不生效(每次都是全部 ALLOW)。
  // 如果要启用，需要让状态在帧间保存(例如存进 Shooter)。
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
  double outpost_fire_window_ = 0.01;   // 前哨站开火相位窗口 rad（配置 outpost_fire_window，缺省 0.01）
  double fire_refractory_ratio_ = 0.5;  // 两次开火最小间隔 = 该比例 × 相邻两块板经过的时间
  std::chrono::steady_clock::time_point last_fire_{};
  // 前哨站定头：跨 tracker 重置保持中心与最高层高度，避免每次重建都重新跟随第一块看到的板
  bool outpost_latched_ = false;
  Eigen::Vector3d outpost_aim_ = Eigen::Vector3d::Zero();
  std::chrono::steady_clock::time_point outpost_seen_{};
  std::chrono::steady_clock::time_point outpost_fired_{};
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
