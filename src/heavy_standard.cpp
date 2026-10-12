// 上车主入口：同济 mt_standard 循环形状，电控换成英雄 0xA5/0x5A（HeroBoard）。
// 本轮仅短焦；自瞄/打符 pitch 经 send() 默认翻号，吊射 send(..., false) 且 20ms 一拍。
#include <chrono>
#include <memory>
#include <thread>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/hero_board.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/lob_shot/command.hpp"
#include "tasks/lob_shot/manager.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | configs/heavy_standard.yaml | yaml }";

using namespace std::chrono_literals;

namespace
{

lob_shot::Config load_lob(const YAML::Node & y)
{
  lob_shot::Config c;
  auto getd = [&](const char * k, double & v) {
    if (y[k]) v = y[k].as<double>();
  };
  auto getb = [&](const char * k, bool & v) {
    if (y[k]) v = y[k].as<bool>();
  };
  getd("target_x", c.target_x);
  getd("target_y", c.target_y);
  getd("target_z", c.target_z);
  getd("base_x", c.base_x);
  getd("base_y", c.base_y);
  getd("base_z", c.base_z);
  getd("base_yaw", c.base_yaw);
  getd("muzzle_dx", c.muzzle_dx);
  getd("muzzle_dy", c.muzzle_dy);
  getd("muzzle_dz", c.muzzle_dz);
  getd("bullet_speed", c.bullet_speed);
  getd("friction", c.friction);
  if (y["friction_coeff"]) c.friction = y["friction_coeff"].as<double>();
  getd("pitch_max_deg", c.pitch_max_deg);
  getd("yaw_tol", c.yaw_tol);
  getd("pitch_tol", c.pitch_tol);
  getd("aim_timeout_s", c.aim_timeout_s);
  getd("pose_timeout_s", c.pose_timeout_s);
  getd("pitch_sign", c.pitch_sign);
  getd("yaw_sign", c.yaw_sign);
  getb("open_loop", c.open_loop);
  getb("module_unlocked", c.module_unlocked);
  getb("module_docked", c.module_docked);
  getb("module_assembling", c.module_assembling);
  getd("windmill_x", c.windmill_x);
  getd("windmill_y", c.windmill_y);
  getd("windmill_radius", c.windmill_radius);
  getd("windmill_z_min", c.windmill_z_min);
  getd("windmill_z_max", c.windmill_z_max);
  return c;
}

}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  io::Camera camera(config_path);
  io::HeroBoard board(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  auto lob_cfg = load_lob(YAML::LoadFile(config_path));
  lob_shot::Manager lob(lob_cfg);
  bool lob_armed = false;

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto last_mode = io::Mode::idle;
  int last_cam = -1;
  auto prev = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    auto mode = board.mode;
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
      if (mode != io::Mode::lob) lob_armed = false;
    }
    if (board.camera_id() != last_cam) {
      last_cam = board.camera_id();
      if (last_cam == 1) {
        tools::logger()->warn("电控切长焦，本轮仅短焦：仍用短焦相机与内参");
      } else {
        tools::logger()->info("英雄相机短焦（上置）");
      }
    }

    // 吊射：不取图，只跑状态机；pitch 已是抬头为正，勿走同济翻号
    if (mode == io::Mode::lob) {
      if (!lob_armed) {
        lob.request_trigger();
        lob_armed = true;
      }
      double jy = 0, jp = 0;
      board.latest_joint(jy, jp);
      lob.set_feedback(jy, lob_shot::joint_pitch_to_nose_up(jp));
      if (board.bullet_speed > 1.0) lob.set_bullet_speed(board.bullet_speed);
      const auto now = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(now - prev).count();
      prev = now;
      auto lob_cmd = lob.tick(dt > 0 ? dt : 0.02);
      io::Command cmd{lob_cmd.control, lob_cmd.shoot, lob_cmd.yaw, lob_cmd.pitch};
      board.send(cmd, false);
      std::this_thread::sleep_for(20ms);
      continue;
    }
    prev = std::chrono::steady_clock::now();

    camera.read(img, t);
    Eigen::Quaterniond q = board.imu_at(t - 1ms);

    if (mode == io::Mode::auto_aim || mode == io::Mode::outpost) {
      if (board.take_reset()) tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
      solver.set_R_gimbal2world(q);
      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      auto armors = detector.detect(img);
      auto targets = tracker->track(armors, t);
      auto command = aimer.aim(targets, t, board.bullet_speed);
      command.shoot = shooter.shoot(command, aimer, targets, ypr);
      board.send(command);  // 同济抬头为负 → 0xA5 翻号
    } else if (mode == io::Mode::small_buff || mode == io::Mode::big_buff) {
      buff_solver.set_R_gimbal2world(q);
      auto power_runes = buff_detector.detect(img);
      buff_solver.solve(power_runes);
      io::Command buff_command;
      if (mode == io::Mode::small_buff) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        buff_command = buff_aimer.aim(target_copy, t, board.bullet_speed, true);
      } else {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        buff_command = buff_aimer.aim(target_copy, t, board.bullet_speed, true);
      }
      board.send(buff_command);  // 同济抬头为负 → 0xA5 翻号
    }
  }

  return 0;
}
