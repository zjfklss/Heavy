#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include "io/mode.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver}, last_timestamp_(std::chrono::steady_clock::now())
{
  //  detect_count_(0), temp_lost_count_(0), state_{"lost"},
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  int min_detect_count = yaml["min_detect_count"].as<int>();
  int max_temp_lost_count = yaml["max_temp_lost_count"].as<int>();
  // x vx y vy z vz a w r1 r2 h
  Eigen::VectorXd P0_dig_car{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
  Eigen::VectorXd P0_dig_outpost{{1, 64, 1, 64, 1, 9, 0.4, 10, 0.0001, 0, 0}};
  Eigen::VectorXd P0_dig_base{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};

  targets_[0] = Target(4, 0.2, P0_dig_car, min_detect_count, max_temp_lost_count, 100, 400);        // hero
  targets_[1] = Target(4, 0.2, P0_dig_car, min_detect_count, max_temp_lost_count, 100, 400);        // engineer
  targets_[2] = Target(4, 0.2, P0_dig_car, min_detect_count, max_temp_lost_count, 100, 400);        // standard 3
  targets_[3] = Target(4, 0.2, P0_dig_car, min_detect_count, max_temp_lost_count, 100, 400);        // standard 4
  targets_[4] = Target(4, 0.2, P0_dig_car, min_detect_count, max_temp_lost_count, 100, 400);        // standard 5
  targets_[5] = Target(4, 0.2, P0_dig_car, min_detect_count, max_temp_lost_count, 100, 400);        // sentry
  // 前哨站每转一圈会有板子短暂不可见(>0.1s)，用默认的 max_temp_lost_count 会每圈都丢失并重置滤波器，导致云台抖动
  const int outpost_temp_lost = yaml["outpost_max_temp_lost_count"] ? yaml["outpost_max_temp_lost_count"].as<int>() : 120;
  targets_[6] = Target(3, 0.2765, P0_dig_outpost, min_detect_count, outpost_temp_lost, 0, 0, 0);  // outpost
  targets_[7] = Target(3, 0.3205, P0_dig_base, min_detect_count, max_temp_lost_count);              // base
}

// std::string Tracker::state() const { return state_; }

// 每帧调用。返回当前要打的目标(0 或 1 个)。
// 两帧间隔超过 0.1 s 视为相机掉线/卡顿，直接清空所有目标。
// 返回策略：刚进入自瞄，或当前目标已 LOST 时，重新选离图像中心最近的装甲板；否则继续返回原目标
// (即使本帧没看到它，只要还没 LOST 也会返回，由 Target 自己外推)。
std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, double yaw, bool use_enemy_color, io::Mode mode)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    clear_targets();
  }

  // 过滤掉我方颜色的装甲板
  if (use_enemy_color) armors.remove_if([&](const Armor & a) { return a.color != enemy_color_; });

  update_targets(armors, t, yaw);

  /* 当前返回策略：
  如果是从idle进入auto_aim，返回距离屏幕中央最近的目标。
  如果是其他状态变化，则判断:若原target没有lost，则返回原目标；如果原target已经lost，则返回距离屏幕中央最近的目标。
  */
  std::list<Target> targets;
  if (((last_mode_ == io::Mode::idle) && (mode == io::Mode::auto_aim)) || (target_.state() == 0)) {  //重新锁定
    armors.sort([](const Armor & a, const Armor & b) {
      auto img_center_norm = cv::Point2f(0.5, 0.5);
      auto distance_1 = cv::norm(a.center_norm - img_center_norm);
      auto distance_2 = cv::norm(b.center_norm - img_center_norm);
      return distance_1 < distance_2;
    });
    if (armors.size()) {  // if at least one armor is detected
      auto armor = armors.front();
      target_ = targets_[armor.name];
      targets = {target_};
      last_target_name_ = armor.name;
    } else {                          // no enemy armor detected
      if (last_target_name_ != -1) {  // there was an target
        target_ = targets_[last_target_name_];
        if (targets_[last_target_name_].state()) {  // not lost yet
          targets = {target_};
        } else {  // already lost;
          targets = {};
        }
      } else {
        targets = {};
      }
    }
  } else {  // 更新当前目标
    target_ = targets_[target_.name];
    if (target_.state()) targets = {target_};
    last_target_name_ = target_.name;
  }
  last_mode_ = mode;
  return targets;
}

void Tracker::clear_targets()
{
  for (auto & target : targets_) {
    target.clear_target();
  }
}

// void Tracker::state_machine(bool found)
// {
//   if (state_ == "lost") {
//     if (!found) return;

//     state_ = "detecting";
//     detect_count_ = 1;
//   }

//   else if (state_ == "detecting") {
//     if (found) {
//       detect_count_++;
//       if (detect_count_ >= min_detect_count_) state_ = "tracking";
//     } else {
//       detect_count_ = 0;
//       state_ = "lost";
//     }
//   }

//   else if (state_ == "tracking") {
//     if (found) return;

//     temp_lost_count_ = 1;
//     state_ = "temp_lost";
//   }

//   else if (state_ == "temp_lost") {
//     if (found) {
//       state_ = "tracking";
//     } else {
//       temp_lost_count_++;
//       if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
//     }
//   }
// }

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t, io::Mode mode)
{
  if (armors.empty()) return false;

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    auto img_center_norm = cv::Point2f(0.5, 0.5);
    auto distance_1 = cv::norm(a.center_norm - img_center_norm);
    auto distance_2 = cv::norm(b.center_norm - img_center_norm);
    return distance_1 < distance_2;
  });

  auto & armor = armors.front();
  solver_.solve(armor);

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four || armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (mode == io::Mode::left_outpost) {
    tools::logger()->debug("mode ####  left");
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 9, 0.4, 0.001, 0.0001, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig, 10, 10, -2.51);  // 仅用于单方向前哨站
  }

  else if (mode == io::Mode::right_outpost) {
    tools::logger()->debug("mode ####  right");
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 9, 0.4, 0.001, 0.0001, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig, 10, 10, 2.51);  // 仅用于单方向前哨站
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 9, 0.4, 0.001, 0.0001, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig, 10, 10, 0);  // 仅用于单方向前哨站
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}

// bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
// {
//   target_.predict(t);

//   int found_count = 0;
//   double best_conf = 0;  // 置信度最大的装甲板
//   for (const auto & armor : armors) {
//     if (armor.name != target_.name || armor.type != target_.armor_type) continue;
//     found_count++;
//     best_conf = armor.confidence > best_conf ? armor.confidence : best_conf;
//   }
//   // tools::logger()->info("best_conf {:.2f}",best_conf);
//   // 现在改成用conf最大的判断
//   // if (found_count > 2)
//   // {
//   // tools::logger()->warn("More than 2 target's armors!");
//   // return false;
//   // }
//   if (found_count == 0) return false;

//   for (auto & armor : armors) {
//     if (armor.name != target_.name || armor.type != target_.armor_type || armor.confidence != best_conf) continue;

//     solver_.solve(armor);

//     target_.update(armor);
//   }

//   return true;
// }

bool Tracker::update_targets(std::list<Armor> & armors, std::chrono::steady_clock::time_point t, double yaw)
{
  for (auto & target : targets_) {
    if (target.state()) {  // 预测所有非空敌方目标的位置。
      target.predict(t);
    }
  }

  // int bestconf[ENEMY_NUM] = {0, 0, 0, 0, 0, 0, 0, 0};
  // armors_use有序存储
  // Armor * armors_use[ENEMY_NUM] = {
  //   NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
  // };  // 每个敌方目标使用至多一块装甲板更新。空指针表示这一帧缺失对应目标。

  // for (auto & armor : armors) {
  //   if (armors_use[armor.name]) {                     // 对应目标已经有装甲板了
  //     if (armor.confidence > bestconf[armor.name]) {  // 挑置信度最大的用
  //       armors_use[armor.name] = &armor;
  //       bestconf[armor.name] = armor.confidence;
  //     }
  //   } else {  // 对应目标还没有装甲板
  //     armors_use[armor.name] = &armor;
  //     bestconf[armor.name] = armor.confidence;
  //   }
  // }

  // int count = 0;
  // for (auto & armor_use : armors_use) {
  //   if (armor_use != NULL) {  // PNP解算，识别到了才解算
  //     solver_.solve(*armor_use);
  //     solver_.optimize_yaw(*armor_use, yaw);
  //   }
  //   targets_[count].update(*armor_use, t);  // 状态更新与跟踪
  //   ++count;
  // }

  std::list<Armor *> armors_use[ENEMY_NUM];
  for (auto & armor : armors) {
    if (armor.name < 0 || armor.name >= ENEMY_NUM) continue;
    armors_use[armor.name].emplace_back(&armor);
    // tools::logger()->debug("detect an armor of target {}", armor.name);
  }
  // tools::logger()->debug("--------------------------------");

  int count = 0;
  for (auto & armor_use : armors_use) {
    if (!armor_use.empty()) {  // PNP解算，识别到了才解算
      // tools::logger()->debug("armors detected for target id:{}", count);
      for (auto & armor : armor_use) {
        solver_.solve(*armor);
        solver_.optimize_yaw(*armor, yaw);
      }
      for (auto & armor : armor_use) {
        targets_[armor->name].update(armor, t);
      }
    } else {
      targets_[count].update(nullptr, t);
    }

    ++count;
  }

  return true;
}

bool Tracker::state() { return target_.state(); }

std::string Tracker::state_str() { return target_.state_str(); }

Target Tracker::get_target() { return target_; }

}  // namespace auto_aim
