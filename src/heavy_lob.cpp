#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

#include "lob_shot/a5_packet.hpp"
#include "lob_shot/gimbal_adapter.hpp"
#include "lob_shot/lob_serial.hpp"
#include "lob_shot/manager.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"

namespace
{

struct RuntimeIo
{
  std::string serial_port = "/dev/ttyUSB0";
  int baud = 921600;
  bool deploy = false;
};

lob_shot::Config load_cfg(const std::string & path, RuntimeIo & io)
{
  lob_shot::Config c;
  YAML::Node y = YAML::LoadFile(path);
  auto getd = [&](const char * k, double & v) {
    if (y[k]) v = y[k].as<double>();
  };
  auto getb = [&](const char * k, bool & v) {
    if (y[k]) v = y[k].as<bool>();
  };
  auto gets = [&](const char * k, std::string & v) {
    if (y[k]) v = y[k].as<std::string>();
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
  // 英雄硬件：与 jlauto-aim/hero serial_driver.yaml 对齐
  gets("serial_port", io.serial_port);
  if (y["device_name"]) io.serial_port = y["device_name"].as<std::string>();
  if (y["baud"]) io.baud = y["baud"].as<int>();
  if (y["baud_rate"]) io.baud = y["baud_rate"].as<int>();
  getb("deploy", io.deploy);
  getb("deploy_mode_enabled", io.deploy);
  return c;
}

lob_shot::Command settle(lob_shot::Manager & mgr)
{
  lob_shot::Command cmd;
  for (int i = 0; i < 60; ++i) {
    cmd = mgr.tick(0.02);
    if (mgr.state() == lob_shot::State::READY) {
      if (mgr.fire_latched()) cmd = mgr.tick(0.02);
      return cmd;
    }
    if (mgr.state() == lob_shot::State::INHIBITED) return cmd;
    if (mgr.status().rfind("IDLE (", 0) == 0) return cmd;
  }
  return cmd;
}

void apply_feedback(lob_shot::Manager & mgr, const lob_shot::Feedback & fb)
{
  if (!fb.ok) return;
  mgr.set_feedback(fb.yaw, fb.pitch);
  if (fb.bullet_speed > 1.0) mgr.set_bullet_speed(fb.bullet_speed);
}

void log_status(const lob_shot::Manager & m, const lob_shot::Command & cmd)
{
  const auto sp = lob_shot::gimbal_sp_bytes(cmd);
  tools::logger()->info(
    "status={} yaw_deg={:.2f} pitch_deg={:.2f} high_unused={:.2f} z={} deploy={} fire={} "
    "gimbal_sp={} yaw_frame=chassis_rel",
    m.status(), m.last_yaw_deg(), m.last_pitch_deg(), m.last_high_deg(), lob_shot::to_z(cmd),
    lob_shot::to_deploy(cmd), m.fire_latched(), lob_shot::to_hex(sp));
}

}  // namespace

int main(int argc, char ** argv)
{
  // 默认串口/波特与英雄 serial_driver.yaml 一致
  const std::string keys =
    "{help h usage ? | | 重装吊射（英雄电控 0xA5/0x5A） }"
    "{@config | configs/heavy_lob.yaml | yaml }"
    "{once | false | 跑到终态退出 }"
    "{fire | false | READY 时开火 }"
    "{undock | false | 模拟未对接 }"
    "{loop | false | 50Hz 循环 }"
    "{dry-run | true | 不写真实串口 }"
    "{deploy | false | 部署模式字节（英雄 deploy_mode_enabled） }"
    "{port | /dev/ttyUSB0 | 串口 }"
    "{baud | 921600 | 波特率 }";

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const auto cfg_path = cli.get<std::string>(0);
  const bool once = cli.get<bool>("once");
  const bool do_fire = cli.get<bool>("fire");
  const bool undock = cli.get<bool>("undock");
  const bool loop = cli.get<bool>("loop");
  const bool dry = cli.get<bool>("dry-run");

  RuntimeIo io;
  lob_shot::Config cfg = load_cfg(cfg_path, io);
  if (cli.has("port")) io.serial_port = cli.get<std::string>("port");
  if (cli.has("baud")) io.baud = cli.get<int>("baud");
  if (cli.has("deploy")) io.deploy = cli.get<bool>("deploy");
  if (undock) cfg.module_docked = false;

  tools::Exiter exiter;
  tools::Plotter plotter;
  lob_shot::Manager mgr(cfg);
  std::unique_ptr<lob_shot::ISender> sender;
  if (dry) {
    sender = std::make_unique<lob_shot::DrySender>();
    tools::logger()->info("sender=DrySender (no hardware)");
  } else {
    auto ser = std::make_unique<lob_shot::SerialSender>(io.serial_port, io.baud);
    if (!ser->open()) {
      tools::logger()->error(
        "open serial {} @{} failed; fallback DrySender", io.serial_port, io.baud);
      sender = std::make_unique<lob_shot::DrySender>();
    } else {
      tools::logger()->info(
        "sender=SerialSender {} @{} (hero 0xA5/0x5A)", io.serial_port, io.baud);
      sender = std::move(ser);
    }
  }

  mgr.request_trigger();
  if (do_fire) mgr.set_fire(true);

  auto stamp_deploy = [&](lob_shot::Command & cmd) { cmd.deploy = io.deploy; };

  if (loop) {
    while (!exiter.exit()) {
      apply_feedback(mgr, sender->read_feedback());
      auto cmd = mgr.tick(0.02);
      stamp_deploy(cmd);
      sender->send(cmd);
      nlohmann::json j = {
        {"status", mgr.status()},
        {"yaw_deg", mgr.last_yaw_deg()},
        {"pitch_deg", mgr.last_pitch_deg()},
        {"z", lob_shot::to_z(cmd)},
        {"deploy", lob_shot::to_deploy(cmd)},
      };
      plotter.plot(j);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
  }

  auto cmd = settle(mgr);
  stamp_deploy(cmd);
  sender->send(cmd);
  log_status(mgr, cmd);

  if (once) {
    if (undock) {
      return (mgr.state() == lob_shot::State::INHIBITED && !cmd.control) ? 0 : 1;
    }
    if (mgr.state() != lob_shot::State::READY) return 2;
    const int z = lob_shot::to_z(cmd);
    if (do_fire) return (z == 2) ? 0 : 3;
    return (z == 1) ? 0 : 4;
  }
  return 0;
}
