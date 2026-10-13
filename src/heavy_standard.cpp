// 上车主入口：同济 mt_standard 循环形状，电控换成英雄 0xA5/0x5A（HeroBoard）。
// 本轮仅短焦；自瞄/打符 pitch 经 send() 默认翻号，吊射 send(..., false) 且 20ms 一拍。
// 中间变量：Plotter UDP→PlotJuggler（不依赖 ROS）。
#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <thread>
#include <nlohmann/json.hpp>
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
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | configs/heavy_standard.yaml | yaml }"
  "{plot p         | true | UDP 推 PlotJuggler（默认 127.0.0.1:9870，无需 ROS）}"
  "{plot-host      | 127.0.0.1 | Plotter 目标 IP }"
  "{plot-port      | 9870 | Plotter UDP 端口 }"
  "{imshow         | true | 显示 raw / vis 窗口（Remmina 桌面看；无显示器加 --imshow=false）}";

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

void put_ekf_debug(nlohmann::json & data, const auto_aim::Target & target)
{
  const auto & dbg = target.ekf().data;
  auto get = [&](const char * k) {
    auto it = dbg.find(k);
    if (it != dbg.end()) data[k] = it->second;
  };
  get("residual_yaw");
  get("residual_pitch");
  get("residual_distance");
  get("residual_angle");
  get("nis");
  get("nees");
  get("nis_fail");
  get("nees_fail");
  get("recent_nis_failures");
}

void show_raw_vis(const cv::Mat & raw, const cv::Mat & vis, int frame, double fps)
{
  if (raw.empty() && vis.empty()) return;
  cv::Mat raw_s, vis_s;
  if (!raw.empty()) {
    cv::resize(raw, raw_s, {}, 0.5, 0.5);
    tools::draw_text(raw_s, fmt::format("raw #{} {:.1f}fps", frame, fps), {10, 30});
    cv::imshow("raw", raw_s);
  }
  if (!vis.empty()) {
    cv::resize(vis, vis_s, {}, 0.5, 0.5);
    tools::draw_text(vis_s, fmt::format("vis #{} {:.1f}fps", frame, fps), {10, 30});
    cv::imshow("vis", vis_s);
  }
  cv::waitKey(1);
}

// 处理前=raw；处理后=装甲框 + 重投影 + 瞄准点（同济 debug 画法）
cv::Mat make_auto_aim_vis(
  const cv::Mat & raw, const std::list<auto_aim::Armor> & armors,
  const std::list<auto_aim::Target> & targets, const auto_aim::Solver & solver,
  const auto_aim::Aimer & aimer, const auto_aim::Tracker & tracker, int frame, double fps)
{
  if (raw.empty()) return {};
  cv::Mat vis = raw.clone();
  tools::draw_text(
    vis, fmt::format("#{} {:.1f}fps [{}]", frame, fps, tracker.state()), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    tools::draw_points(vis, armor.points, {0, 255, 0});
  }
  if (!targets.empty()) {
    const auto & target = targets.front();
    for (const Eigen::Vector4d & xyza : target.armor_xyza_list()) {
      auto pts = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
      tools::draw_points(vis, pts, {0, 255, 0});
    }
    const auto & aim = aimer.debug_aim_point;
    if (aim.valid) {
      auto pts =
        solver.reproject_armor(aim.xyza.head(3), aim.xyza[3], target.armor_type, target.name);
      tools::draw_points(vis, pts, {0, 0, 255});
    }
  }
  return vis;
}

// 对齐 uav_debug / mt_auto_aim_debug + 本队英雄字段
nlohmann::json plot_auto_aim(
  double t_s, io::Mode mode, const cv::Mat & img, const Eigen::Vector3d & ypr,
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets,
  const auto_aim::Solver & solver, const auto_aim::Tracker & tracker, const auto_aim::Aimer & aimer,
  const io::Command & command, const io::HeroBoard & board, int frame, double fps)
{
  nlohmann::json data;
  data["t"] = t_s;
  data["frame"] = frame;
  data["fps"] = fps;
  data["mode"] = io::MODES[mode];
  data["cam_id"] = board.camera_id();
  data["img_empty"] = img.empty();
  data["img_w"] = img.empty() ? 0 : img.cols;
  data["img_h"] = img.empty() ? 0 : img.rows;
  data["tracker_state"] = tracker.state();
  data["armor_num"] = static_cast<int>(armors.size());
  data["target_num"] = static_cast<int>(targets.size());

  data["gimbal_yaw"] = ypr[0] * 57.3;
  data["gimbal_pitch"] = ypr[1] * 57.3;
  data["gimbal_roll"] = ypr[2] * 57.3;
  double jy = 0, jp = 0;
  board.latest_joint(jy, jp);
  data["joint_yaw"] = jy * 57.3;
  data["joint_pitch"] = jp * 57.3;  // 英雄 URDF，抬头为正
  data["bullet_speed"] = board.bullet_speed;

  data["cmd_yaw"] = command.yaw * 57.3;
  data["cmd_pitch"] = command.pitch * 57.3;          // 同济世界系：抬头为负
  data["cmd_pitch_hero"] = (-command.pitch) * 57.3;  // HeroBoard 翻号后下发
  data["cmd_control"] = command.control;
  data["cmd_shoot"] = command.shoot;

  data["nail_mode"] = aimer.nail_mode();
  data["nail_latched"] = aimer.nail_latched();
  data["aim_valid"] = aimer.debug_aim_point.valid;
  if (aimer.debug_aim_point.valid) {
    const auto & xyza = aimer.debug_aim_point.xyza;
    data["aim_x"] = xyza[0];
    data["aim_y"] = xyza[1];
    data["aim_z"] = xyza[2];
    data["aim_yaw"] = xyza[3] * 57.3;
  }

  if (!armors.empty()) {
    auto armor = armors.front();
    double min_x = armor.center.x;
    for (const auto & a : armors) {
      if (a.center.x < min_x) {
        min_x = a.center.x;
        armor = a;
      }
    }
    solver.solve(armor);
    data["armor_x"] = armor.xyz_in_world[0];
    data["armor_y"] = armor.xyz_in_world[1];
    data["armor_z"] = armor.xyz_in_world[2];
    data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    data["armor_cx"] = armor.center.x;
    data["armor_cy"] = armor.center.y;
  }

  if (!targets.empty()) {
    const auto & target = targets.front();
    Eigen::VectorXd x = target.ekf_x();
    data["x"] = x[0];
    data["vx"] = x[1];
    data["y"] = x[2];
    data["vy"] = x[3];
    data["z"] = x[4];
    data["vz"] = x[5];
    data["a"] = x[6] * 57.3;
    data["w"] = x[7];
    data["abs_w"] = std::abs(x[7]);
    data["r"] = x[8];
    data["l"] = x[9];
    data["h"] = x[10];
    data["last_id"] = target.last_id;
    data["jumped"] = target.jumped;
    put_ekf_debug(data, target);
  }
  return data;
}

nlohmann::json plot_lob(
  double t_s, io::Mode mode, const lob_shot::Manager & lob, const lob_shot::Command & lob_cmd,
  const io::HeroBoard & board, double joint_yaw, double joint_pitch, int frame, double fps)
{
  nlohmann::json data;
  data["t"] = t_s;
  data["frame"] = frame;
  data["fps"] = fps;
  data["mode"] = io::MODES[mode];
  data["cam_id"] = board.camera_id();
  data["status"] = lob.status();
  data["yaw_deg"] = lob.last_yaw_deg();
  data["pitch_deg"] = lob.last_pitch_deg();
  data["high_unused_deg"] = lob.last_high_deg();
  data["z"] = lob_shot::to_z(lob_cmd);
  data["deploy"] = lob_shot::to_deploy(lob_cmd);
  data["fire_latched"] = lob.fire_latched();
  data["cmd_yaw"] = lob_cmd.yaw * 57.3;
  data["cmd_pitch"] = lob_cmd.pitch * 57.3;  // 英雄抬头为正
  data["cmd_control"] = lob_cmd.control;
  data["cmd_shoot"] = lob_cmd.shoot;
  data["joint_yaw"] = joint_yaw * 57.3;
  data["joint_pitch"] = joint_pitch * 57.3;
  data["bullet_speed"] = board.bullet_speed;
  return data;
}

nlohmann::json plot_buff(
  double t_s, io::Mode mode, const io::Command & cmd, const io::HeroBoard & board, bool unsolve,
  int frame, double fps)
{
  nlohmann::json data;
  data["t"] = t_s;
  data["frame"] = frame;
  data["fps"] = fps;
  data["mode"] = io::MODES[mode];
  data["cam_id"] = board.camera_id();
  data["buff_unsolve"] = unsolve;
  data["cmd_yaw"] = cmd.yaw * 57.3;
  data["cmd_pitch"] = cmd.pitch * 57.3;
  data["cmd_pitch_hero"] = (-cmd.pitch) * 57.3;
  data["cmd_control"] = cmd.control;
  data["cmd_shoot"] = cmd.shoot;
  data["bullet_speed"] = board.bullet_speed;
  double jy = 0, jp = 0;
  board.latest_joint(jy, jp);
  data["joint_yaw"] = jy * 57.3;
  data["joint_pitch"] = jp * 57.3;
  return data;
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
  const bool do_plot = cli.get<bool>("plot");
  std::unique_ptr<tools::Plotter> plotter;
  if (do_plot) {
    plotter = std::make_unique<tools::Plotter>(
      cli.get<std::string>("plot-host"), static_cast<uint16_t>(cli.get<int>("plot-port")));
    tools::logger()->info(
      "Plotter UDP {}:{}（PlotJuggler 监听即可，无需 ROS）", cli.get<std::string>("plot-host"),
      cli.get<int>("plot-port"));
  }
  const bool do_imshow = cli.get<bool>("imshow");
  if (do_imshow) {
    tools::logger()->info("imshow=raw+vis（处理前/后）；无桌面时请 --imshow=false");
  }

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
  const auto t0 = prev;
  int frame = 0;
  double fps = 0;
  auto fps_t0 = prev;
  int fps_n = 0;

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
      ++frame;
      ++fps_n;
      const auto now_fps = std::chrono::steady_clock::now();
      const double fps_dt = std::chrono::duration<double>(now_fps - fps_t0).count();
      if (fps_dt >= 0.5) {
        fps = fps_n / fps_dt;
        fps_n = 0;
        fps_t0 = now_fps;
      }
      if (plotter) {
        plotter->plot(plot_lob(
          tools::delta_time(std::chrono::steady_clock::now(), t0), mode, lob, lob_cmd, board, jy, jp,
          frame, fps));
      }
      std::this_thread::sleep_for(20ms);
      continue;
    }
    prev = std::chrono::steady_clock::now();

    camera.read(img, t);
    ++frame;
    ++fps_n;
    {
      const auto now_fps = std::chrono::steady_clock::now();
      const double fps_dt = std::chrono::duration<double>(now_fps - fps_t0).count();
      if (fps_dt >= 0.5) {
        fps = fps_n / fps_dt;
        fps_n = 0;
        fps_t0 = now_fps;
      }
    }
    const cv::Mat raw = img.empty() ? cv::Mat() : img.clone();
    Eigen::Quaterniond q = board.imu_at(t - 1ms);

    if (mode == io::Mode::auto_aim || mode == io::Mode::outpost) {
      if (board.take_reset()) tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
      solver.set_R_gimbal2world(q);
      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      auto armors = detector.detect(img, frame);
      auto targets = tracker->track(armors, t);
      auto command = aimer.aim(targets, t, board.bullet_speed);
      command.shoot = shooter.shoot(command, aimer, targets, ypr);
      board.send(command);  // 同济抬头为负 → 0xA5 翻号
      if (plotter) {
        plotter->plot(plot_auto_aim(
          tools::delta_time(std::chrono::steady_clock::now(), t0), mode, img, ypr, armors, targets,
          solver, *tracker, aimer, command, board, frame, fps));
      }
      if (do_imshow) {
        show_raw_vis(
          raw, make_auto_aim_vis(raw, armors, targets, solver, aimer, *tracker, frame, fps), frame,
          fps);
      }
    } else if (mode == io::Mode::small_buff || mode == io::Mode::big_buff) {
      buff_solver.set_R_gimbal2world(q);
      auto power_runes = buff_detector.detect(img);
      buff_solver.solve(power_runes);
      io::Command buff_command;
      bool unsolve = true;
      if (mode == io::Mode::small_buff) {
        buff_small_target.get_target(power_runes, t);
        auto target_copy = buff_small_target;
        unsolve = target_copy.is_unsolve();
        buff_command = buff_aimer.aim(target_copy, t, board.bullet_speed, true);
      } else {
        buff_big_target.get_target(power_runes, t);
        auto target_copy = buff_big_target;
        unsolve = target_copy.is_unsolve();
        buff_command = buff_aimer.aim(target_copy, t, board.bullet_speed, true);
      }
      board.send(buff_command);  // 同济抬头为负 → 0xA5 翻号
      if (plotter) {
        auto data = plot_buff(
          tools::delta_time(std::chrono::steady_clock::now(), t0), mode, buff_command, board,
          unsolve, frame, fps);
        data["rune_ok"] = power_runes.has_value();
        plotter->plot(data);
      }
      if (do_imshow && !raw.empty()) {
        cv::Mat vis = raw.clone();
        tools::draw_text(
          vis, fmt::format("#{} {:.1f}fps buff unsolve={}", frame, fps, unsolve), {10, 30});
        show_raw_vis(raw, vis, frame, fps);
      }
    }
  }

  return 0;
}
