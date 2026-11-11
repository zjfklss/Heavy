#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <nlohmann/json.hpp>
#include <algorithm>
#include <limits>

#include "tasks/auto_aim/armor.hpp"
#include "tools/math_tools.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"
#include "tools/trajectory.hpp"
// 选层规则见 shooter.hpp 的声明注释。实现：先按高度降序，最高两层足够分开就直接取最高层；
// 否则(高度差 < threshold)在这些层里取与当前 pitch 最接近的一层。
namespace auto_outpost
{
int select_layer(const std::vector<Eigen::Vector4d> & armors, double current_pitch, double threshold)
{
  std::vector<size_t> valid;
  for (size_t index = 0; index < armors.size(); ++index)
    if (armors[index].allFinite() && std::abs(armors[index][2]) > 0.01) valid.push_back(index);
  if (valid.empty()) return -1;
  std::stable_sort(valid.begin(), valid.end(), [&](size_t first, size_t second) {
    return armors[first][2] > armors[second][2];
  });
  size_t selected = valid.front();
  if (valid.size() > 1 && armors[valid[0]][2] - armors[valid[1]][2] + 1e-9 < threshold) {
    double nearest = std::numeric_limits<double>::infinity();
    for (size_t index : valid) {
      const double pitch = std::atan2(armors[index][2], armors[index].head<2>().norm());
      const double difference = std::abs(pitch - current_pitch);
      if (difference < nearest) { nearest = difference; selected = index; }
    }
  }
  return static_cast<int>(selected);
}

Shooter::Shooter(const std::string & config_path) : exit_{false}, queue_0_(500), queue_1_(500), queue_2_(500)
{
  auto yaml = YAML::LoadFile(config_path);
  ctrl_to_fire_ = yaml["shoot_delay"].as<double>();
  mouse_to_fire_ = yaml["mouse_to_fire"].as<double>();
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  direction_ = yaml["direction"].as<double>();               // pitch正方向。气动为1，摩擦轮为-1
  drag_ = yaml["k_air_resistance"].as<double>();
  layer_threshold_ = yaml["layer_height_threshold"].as<double>();
  debug_aim_point_ = {false, Eigen::Vector4d::Zero()};
  max_fire_error_yaw_ = yaml["max_fire_error_yaw"].as<double>();
  max_fire_error_pitch_ = yaml["max_fire_error_pitch"].as<double>();
  max_shoot_angle_ = yaml["max_shoot_angle"].as<double>();
  if (yaml["outpost_fire_window"]) outpost_fire_window_ = yaml["outpost_fire_window"].as<double>();
  if (yaml["fire_refractory_ratio"]) fire_refractory_ratio_ = yaml["fire_refractory_ratio"].as<double>();
}

// 对于outpost，我们要打的位置AimPoint
//
// 返回云台应该瞄的世界系点 {x, y, z}：旋转中心朝向我们一侧的"表面"(中心水平距离减去半径)，高度取最高层装甲板。
// 前哨站是固定不动的，所以这里刻意做成"定头"，并且为了云台不抖做了三件事：
//   1) 恒打最高层(select_layer 阈值传 0)，不按当前 pitch 就近换层。
//   2) 瞄准点锁存(outpost_aim_)：tracker 每次重置都会用单块板重新初始化中心和层高，
//      直接用会让云台去追"第一块看到的板"。已有锁定后只在已知层数>=2(估计已收敛)时缓慢修正(每帧1%)。
//   3) 发射后 0.8 s 内不更新：后坐力会让云台/相机晃动，世界系估计随之变化，指令再跟着变就形成回路。
// 例外：估计与锁定点相差太大(水平>0.3m 或高度>0.2m，例如车体移动或换了前哨站)时直接重新锁定；
// 连续 3 s 没有前哨站则释放锁定。
Eigen::Vector3d Shooter::get_outpost_front(const auto_aim::Target & target_origin, double current_pitch)
{
  auto ekf_x = target_origin.ekf_x();

  Eigen::Vector3d center = {ekf_x[0], ekf_x[2], ekf_x[4]};
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);  // 整车旋转中心的球坐标yaw

  Eigen::Vector2d center_xy = center.head(2);
  double horizonal_distance = center_xy.norm() - ekf_x[8];
  Eigen::Vector2d hit_point_xy = (horizonal_distance / center_xy.norm()) * center_xy;
  auto armors = target_origin.armor_xyza_list();
  const auto & heights = target_origin.layer_heights();
  for (size_t index = 0; index < armors.size(); ++index)
    if (index >= heights.size() || !heights[index]) armors[index][2] = std::numeric_limits<double>::quiet_NaN();
  size_t known = 0;
  for (const auto & armor : armors) if (std::isfinite(armor[2])) ++known;
  const int selected = select_layer(armors, current_pitch, 0.0);  // 定头：恒打最高层，避免按当前pitch就近选层的正反馈抖动
  if (selected < 0) {
    if (outpost_latched_) return outpost_aim_;
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  const Eigen::Vector3d fresh{hit_point_xy[0], hit_point_xy[1], armors[selected][2]};
  const auto now = decide_time_ == std::chrono::steady_clock::time_point{} ? std::chrono::steady_clock::now() : decide_time_;
  if (!outpost_latched_ || now - outpost_seen_ > std::chrono::seconds(3)) {
    outpost_aim_ = fresh;  // 首次或丢失太久：重新锁定
    outpost_latched_ = true;
  } else if (
    known >= 2 && ((fresh.head<2>() - outpost_aim_.head<2>()).norm() > 0.3 || std::abs(fresh[2] - outpost_aim_[2]) > 0.2)) {
    outpost_aim_ = fresh;  // 估计与锁定点差距过大(车体移动或换了前哨站)：直接重新锁定
  } else if (known >= 2 && now - outpost_fired_ > std::chrono::milliseconds(800)) {
    // 前哨站不动：只在看到>=2层(估计已收敛)时缓慢修正，tracker重置后的单板估计不会拉偏云台。
    // 发射后0.8s冻结：后坐力会让云台/相机晃动，世界系估计随之抖，指令再跟着动就形成回路
    constexpr double alpha = 0.01;
    outpost_aim_.head<2>() += alpha * (fresh.head<2>() - outpost_aim_.head<2>());
    outpost_aim_[2] += alpha * (fresh[2] - outpost_aim_[2]);  // 不做只升不降的棘轮：高度测量有噪声，会越抬越高
  }
  outpost_seen_ = now;
  return outpost_aim_;
}

// 对于小陀螺的车，我们要打的位置AimPoint
Eigen::Vector3d Shooter::get_top_front(const auto_aim::Target & target_origin)
{
  // 这时候瞄准位置的z坐标是下一块来接紫蛋的装甲板的高度
  auto ekf_x = target_origin.ekf_x();

  Eigen::Vector3d center = {ekf_x[0], ekf_x[2], ekf_x[4]};
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);  // 整车旋转中心的球坐标yaw

  Eigen::Vector2d center_xy = center.head(2);
  double horizonal_distance = center_xy.norm() - ekf_x[8];
  Eigen::Vector2d hit_point_xy = (horizonal_distance / center_xy.norm()) * center_xy;

  auto armors = target_origin.armor_xyza_list();
  int armor_num = armors.size();

  double w = ekf_x[7], T = 2 * M_PI / w;
  int sig_w = w > 0 ? 1 : -1;  // 旋转方向
  int armor_id = -1;
  double dt_min = INFINITY, t;
  if (std::fabs(w) > 3) {
    return {hit_point_xy[0], hit_point_xy[1], ekf_x[4]};
  }
  for (int aim_id = 0; aim_id < armor_num; aim_id++) {
    if (sig_w == 1) {  // w>0
      t = (armors[aim_id][3] > center_yaw) ? T - (armors[aim_id][3] - center_yaw) / w : (center_yaw - armors[aim_id][3] / w);
    } else {
      t = (armors[aim_id][3] < center_yaw) ? T + (armors[aim_id][3] - center_yaw) / w : (-center_yaw + armors[aim_id][3] / w);
    }
    if (t < dt_min) {
      armor_id = aim_id;
      dt_min = t;
    }
  }
  return {hit_point_xy[0], hit_point_xy[1], armors[armor_id][2]};
}

// 主入口：根据当前目标算出这一帧要发给云台的指令。
//   targets    : tracker 输出的目标，空表示没有目标(返回 control=false，云台保持)。
//   timestamp  : 这帧图像的时间戳。
//   bullet_speed: 弹速 m/s，超出 (10, 20) 视为反馈异常，不出指令。
//   to_now     : true 表示实机(按"现在"外推延迟)；false 用于离线回放/测试(按图像时间戳算)。
//   ypr        : 当前云台姿态，仅平动分支用来判断是否已对准。
// 返回的 io::Command 四个字段：{control 是否控制云台, shoot 是否开火, yaw, pitch}，角度单位 rad。
io::Command Shooter::shoot(
  std::list<auto_aim::Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed, bool to_now,
  Eigen::Vector3d ypr)
{
  static thread_local tools::Plotter plotter;

  // 没有有效目标时：
  if (targets.empty()) {
    last_hit_id_ = -1;
    lock_id_ = -1;
    if (outpost_latched_ && timestamp - outpost_seen_ > std::chrono::seconds(3)) outpost_latched_ = false;
    debug_aim_point_.valid = false;
    return {false, false, 0, 0};
  }

  // 得到有效目标时
  auto target = targets.front();
  auto target_predicted = target;
  auto t_img = timestamp;
  auto t_fire = timestamp;
  if (!std::isfinite(bullet_speed) || bullet_speed <= 10 || bullet_speed >= 20) return {false, false, 0, 0};
  auto ekf_x = target_predicted.ekf_x();

  if (std::abs(ekf_x[7]) > 1 || target.name == auto_aim::outpost) {  // w 大于1就认为在旋转；前哨站一律定头
    auto target_rotate = target;
    // 程序运行到这一句的时刻
    auto t_decide = to_now ? std::chrono::steady_clock::now() : t_img;
    target_rotate.predict(t_decide);  // 补上延迟，主要是神经网络
    ekf_x = target_rotate.ekf_x();

    Eigen::Vector3d xyz0;
    decide_time_ = t_decide;

    if (target.name == auto_aim::outpost) {  // 反前哨站。不存在高低装甲板
      // tools::logger()->info("anti outpost mode");
      xyz0 = get_outpost_front(target, -ypr[1]);  // 定头：用未外推的中心，瞄准点不随速度估计抖动
    } else {                                    // 反小陀螺，需要应对高低装甲板
      // tools::logger()->info("anti top mode");
      xyz0 = get_top_front(target_rotate);  // 瞄准点
    }

    if (!xyz0.allFinite()) {  // 没有任何有效层高度：不输出 NaN 指令
      debug_aim_point_.valid = false;
      return {false, false, 0, 0};
    }
    double yaw = std::atan2(xyz0[1], xyz0[0]) + yaw_offset_;  // yaw直接瞄准旋转中心

    double d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
    tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2], drag_);
    if (trajectory0.unsolvable) {
      tools::logger()->debug("[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
      return {false, false, 0, 0};
    }
    auto pitch = trajectory0.pitch + pitch_offset_;  // pitch瞄准装甲板正对时的位置

    debug_aim_point_ = {true, {xyz0[0], xyz0[1], xyz0[2], 0}};

    auto t_hit = tools::add_time(t_decide, (ctrl_to_fire_ + trajectory0.fly_time));
    target_rotate.predict(t_hit);  // 预测如果这时候发射，紫蛋到达时的情况
    auto armors_hit = target_rotate.armor_xyza_list();
    int armor_num = armors_hit.size();
    bool good_converge = target_rotate.ekf().last_nis < 0.484;
    int sig = ekf_x[7] < 0 ? -1 : +1;
    double error = (target_rotate.name == auto_aim::ArmorName::outpost ? outpost_fire_window_ : (std::fabs(ekf_x[7]) > 5 ? 0.04 : 0.02));
    auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
    auto armor_state = target.armor_state;
    nlohmann::json data;
    // 开火判定的核心：云台固定瞄中心正前方，那么"哪块板的朝向角恰好等于中心方位角"，就是它正对我们。
    // 把目标预测到弹丸命中时刻(t_hit = 现在 + 开火延迟 + 飞行时间)，对每块板算
    //   phase_error = 旋转方向符号 * (中心方位角 - 该板朝向角)
    // 它从正数减小到 0 的过程就是"板子正在转到枪口前"，所以只在 0 <= phase_error <= error 时开火：
    // 弹丸到达时板子刚好转正。符号取决于旋转方向 sig，因此转向(w 的符号)必须是对的，反了就会整个窗口错位，
    // 表现为命中率骤降(见 Target::monitor_outpost_direction)。
    // error 前哨站默认 0.01 rad(约 0.57°)，可用配置 outpost_fire_window 调整。
    for (int aim_id = 0; aim_id < armor_num; aim_id++) {
      if (GET_STATE(armor_state, aim_id) == ALLOW) {
        const double phase_error = sig * tools::limit_rad(center_yaw - armors_hit[aim_id][3]);
        if (
          (phase_error <= error) && (phase_error >= 0) &&
          (std::fabs(armors_hit[aim_id][2] - xyz0[2]) <= (target.name == auto_aim::outpost ? 0.04 : 0.01))) {  // 在击打窗口内
          if (last_hit_id_ != aim_id)
            tools::logger()->info("[Shooter] fire window entered: armor={} phase_error={:.6f}", aim_id, phase_error);
          last_hit_id_ = aim_id;
          {  // 防连发：窗口很窄但决策线程每毫秒跑一次，同一块板经过时可能连续两帧都满足，这里按板间隔去抖
            const double pass_time = (2 * M_PI / armor_num) / std::max(std::fabs(ekf_x[7]), 0.5);
            if (std::chrono::duration<double>(t_decide - last_fire_).count() <
                fire_refractory_ratio_ * pass_time)
              continue;
          }
          if (target.name == auto_aim::outpost && target.temp_lost_count() > 60) continue;  // 约0.75s没看到任何板：不盲打
          last_fire_ = t_decide;
          if (target.name == auto_aim::outpost) outpost_fired_ = t_decide;
          io::Command command = {true, true, yaw, direction_ * pitch};
          // data["shoot"] = 1;
          // plotter.plot(data);
          if (target_rotate.name == auto_aim::outpost) {
            armor_state = 0;
            SET_STATE(armor_state, aim_id, SHOOTED);
            SET_STATE(armor_state, (aim_id - sig + armor_num) % armor_num, SKIP);
          }
          return command;
        }
      } else if (GET_STATE(armor_state, aim_id) == SKIP) {  // 上一块刚打过
        tools::logger()->info("---------- wait ----------");
        if (target_rotate.name == auto_aim::outpost) {
          armor_state = 0;
          SET_STATE(armor_state, aim_id, SHOOTED);
        }
      }
    }
    last_hit_id_ = -1;
    io::Command command = {true, false, yaw, direction_ * pitch};
    data["shoot"] = 0;
    plotter.plot(data);
    return command;
  }  // end of anti top/outpost
  else {  // 平动目标
    if (to_now) {
      auto dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + mouse_to_fire_;
      t_fire = tools::add_time(t_img, dt);
      // t_img + std::chrono::microseconds(int(dt * 1e6));  意义不明的一句 ???
      target_predicted.predict(t_fire);  // 预测紫蛋出膛时的目标状态
    }

    auto aim_point0 = choose_aim_point(target_predicted, -ypr[1]);
    debug_aim_point_ = aim_point0;
    if (!aim_point0.valid) {
      // tools::logger()->debug("Invalid aim_point0.");
      io::Command command = {false, false, 0, 0};
      return command;
    }

    Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
    auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
    tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2], drag_);
    if (trajectory0.unsolvable) {
      tools::logger()->debug("[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
      debug_aim_point_.valid = false;
      io::Command command = {false, false, 0, 0};
      return command;
    }

    auto t_hit = tools::add_time(t_fire, trajectory0.fly_time);
    target_predicted.predict(t_hit);

    auto aim_point1 = choose_aim_point(target_predicted, -ypr[1]);
    debug_aim_point_ = aim_point1;
    if (!aim_point1.valid) {
      // tools::logger()->debug("Invalid aim_point1.");
      io::Command command = {false, false, 0, 0};
      return command;
    }

    Eigen::Vector3d xyz1 = aim_point1.xyza.head(3);
    auto d1 = std::sqrt(xyz1[0] * xyz1[0] + xyz1[1] * xyz1[1]);
    tools::Trajectory trajectory1(bullet_speed, d1, xyz1[2], drag_);
    if (trajectory1.unsolvable) {
      tools::logger()->debug("[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d1, xyz1[2]);
      debug_aim_point_.valid = false;
      io::Command command = {false, false, 0, 0};
      return command;
    }

    auto time_error = trajectory1.fly_time - trajectory0.fly_time;
    if (std::abs(time_error) > 0.1) {
      tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
      debug_aim_point_.valid = false;
      io::Command command = {false, false, 0, 0};
      return command;
    }

    double yaw;
    yaw = std::atan2(xyz1[1], xyz1[0]) + yaw_offset_;
    // tools::logger()->debug("xyz1[0] = {}, xyz1[1] = {}", xyz1[0], xyz1[1]);

    auto pitch = trajectory1.pitch + pitch_offset_;

    bool is_fire = false;
    if ((std::abs(ypr[0] - yaw) < max_fire_error_yaw_) && (std::abs(ypr[1] - direction_ * pitch) < max_fire_error_pitch_)) {
      // is_fire = true;
      // tools::logger()->info("########## fire ##########");
    }

    io::Command command = {true, is_fire, yaw, direction_ * pitch};

    return command;
  }
}

// AimPoint Shooter::choose_aim_point(const auto_aim::Target & target)
// {
//   Eigen::VectorXd ekf_x = target.ekf_x();
//   std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
//   auto armor_num = armor_xyza_list.size();

//   // 整车旋转中心的球坐标yaw
//   auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

//   int armor_id;
//   int first_id = -1, second_id = -1;
//   float min1 = std::numeric_limits<float>::max();
//   float min2 = std::numeric_limits<float>::max();

//   for (int aim_id = 0; aim_id < armor_num; aim_id++) {
//       float delta = std::abs(armor_xyza_list[aim_id][3] - center_yaw);

//       if (delta < min1) {
//           // 更新第一小的值，同时把原来的第一小的值变成第二小的值
//           min2 = min1;
//           second_id = first_id;

//           min1 = delta;
//           first_id = aim_id;
//       } else if (delta < min2) {
//           // 仅更新第二小的值
//           min2 = delta;
//           second_id = aim_id;
//       }
//   }

//   Eigen::Vector3d xyz1 = armor_xyza_list[first_id].head(3);
//   Eigen::Vector3d xyz2 = armor_xyza_list[second_id].head(3);

//   double yaw1 = std::atan2(xyz1[1], xyz1[0]);
//   double yaw2 = std::atan2(xyz2[1], xyz2[0]);

//   armor_id = (std::abs(armor_xyza_list[first_id][3]-yaw1) < std::abs(armor_xyza_list[second_id][3]-yaw2)) ? first_id : second_id;

//   return {true, armor_xyza_list[armor_id]};
// }

// 平动目标选瞄准点(定头分支不走这里)。前哨站直接取最高层；普通车则在"最正对我们的两块板"里选，
// 并用 lock_id_ 锁定，避免两块板都呈 45° 时在它们之间来回切换。
AimPoint Shooter::choose_aim_point(const auto_aim::Target & target, double current_pitch)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();

  if (target.name == auto_aim::outpost) {
    const auto & heights = target.layer_heights();
    for (size_t index = 0; index < armor_xyza_list.size(); ++index)
      if (index >= heights.size() || !heights[index]) armor_xyza_list[index][2] = std::numeric_limits<double>::quiet_NaN();
    const int selected = select_layer(armor_xyza_list, current_pitch, 0.0);  // 定头：恒打最高层
    return selected >= 0 ? AimPoint{true, armor_xyza_list[selected]} : AimPoint{false, Eigen::Vector4d::Zero()};
  }

  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  int first_id = -1, second_id = -1;
  float min1 = std::numeric_limits<float>::max();
  float min2 = std::numeric_limits<float>::max();

  for (int aim_id = 0; aim_id < armor_num; aim_id++) {
    float delta = std::abs(armor_xyza_list[aim_id][3] - center_yaw);

    if (delta < min1) {
      // 更新第一小的值，同时把原来的第一小的值变成第二小的值
      min2 = min1;
      second_id = first_id;

      min1 = delta;
      first_id = aim_id;
    } else if (delta < min2) {
      // 仅更新第二小的值
      min2 = delta;
      second_id = aim_id;
    }
  }

  int dmin_first_id = -1, dmin_second_id = -1;
  float dmin1 = std::numeric_limits<float>::max();
  float dmin2 = std::numeric_limits<float>::max();

  for (int aim_id = 0; aim_id < armor_num; aim_id++) {
    float delta =
      armor_xyza_list[aim_id][0] * armor_xyza_list[aim_id][0] + armor_xyza_list[aim_id][1] * armor_xyza_list[aim_id][1];

    if (delta < dmin1) {
      // 更新第一小的值，同时把原来的第一小的值变成第二小的值
      dmin2 = dmin1;
      dmin_second_id = dmin_first_id;

      dmin1 = delta;
      dmin_first_id = aim_id;
    } else if (delta < dmin2) {
      // 仅更新第二小的值
      dmin2 = delta;
      dmin_second_id = aim_id;
    }
  }

  Eigen::Vector3d xyz1 = armor_xyza_list[first_id].head(3);
  Eigen::Vector3d xyz2 = armor_xyza_list[second_id].head(3);

  double yaw1 = std::atan2(xyz1[1], xyz1[0]);
  double yaw2 = std::atan2(xyz2[1], xyz2[0]);

  if (first_id != dmin_first_id && first_id != dmin_second_id) return {true, armor_xyza_list[second_id]};

  if (second_id != dmin_first_id && second_id != dmin_second_id) return {true, armor_xyza_list[first_id]};

  if (std::abs(armor_xyza_list[first_id][3] - yaw1) > max_shoot_angle_ / 57.3) return {true, armor_xyza_list[second_id]};

  if (std::abs(armor_xyza_list[second_id][3] - yaw2) > max_shoot_angle_ / 57.3) return {true, armor_xyza_list[first_id]};

  // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
  // 未处于锁定模式时，选择delta_angle绝对值较小（更“正对”机器人）的装甲板，进入锁定模式
  if (lock_id_ != first_id && lock_id_ != second_id)
    lock_id_ =
      (std::abs(armor_xyza_list[first_id][3] - yaw1) < std::abs(armor_xyza_list[second_id][3] - yaw2)) ? first_id : second_id;

  return {true, armor_xyza_list[lock_id_]};
}

// AimPoint Shooter::choose_aim_point(const auto_aim::Target & target)
// {
//   Eigen::VectorXd ekf_x = target.ekf_x();
//   std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
//   auto armor_num = armor_xyza_list.size();

//   // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
//   if (!target.jumped) return {true, armor_xyza_list[0]};

//   // 整车旋转中心的球坐标yaw
//   auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

//   // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
//   std::vector<double> delta_angle_list;
//   for (int i = 0; i < 3; i++) {
//     auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
//     delta_angle_list.emplace_back(delta_angle);
//   }

//   // 选择在可射击范围内的装甲板
//   std::vector<int> id_list;
//   // tools::logger()->debug("-------------------------------------------");
//   for (int i = 0; i < armor_num; i++) {
//     if (std::abs(delta_angle_list[i]) > 60 / 57.3) {
//       // tools::logger()->debug(std::to_string(std::abs(delta_angle_list[i] * 57.3)));
//       continue;  // 以60度为射击范围
//     }
//     id_list.push_back(i);
//   }
//   // tools::logger()->debug("-------------------------------------------");

//   if (id_list.empty()) {
//     tools::logger()->warn("Empty id list in Shooter!");
//     return {false, armor_xyza_list[0]};
//   }

//   // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
//   if (id_list.size() > 1) {
//     int id0 = id_list[0], id1 = id_list[1];

//     // 未处于锁定模式时，选择delta_angle绝对值较小（更“正对”机器人）的装甲板，进入锁定模式
//     if (lock_id_ != id0 && lock_id_ != id1)
//       lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

//     return {true, armor_xyza_list[lock_id_]};
//   }

//   // 只有一个装甲板在可射击范围内时，退出锁定模式
//   lock_id_ = -1;
//   return {true, armor_xyza_list[id_list[0]]};
// }

Shooter::~Shooter()
{
  exit_ = true;
  if (when_to_fire_.joinable()) {
    when_to_fire_.join();
  }
  tools::logger()->info("Shooter destructed");
}

}  // namespace auto_outpost
