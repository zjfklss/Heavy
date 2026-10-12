#include <cmath>
#include <cstring>
#include <iostream>

#include "lob_shot/a5_packet.hpp"
#include "lob_shot/gimbal_adapter.hpp"
#include "lob_shot/manager.hpp"
#include "projectile_motion/gaf_projectile_solver.hpp"
#include "tools/crc.hpp"

namespace
{
bool near(double a, double b, double eps) { return std::fabs(a - b) < eps; }
int fail(const char * msg)
{
  std::cerr << "FAIL: " << msg << "\n";
  return 1;
}

lob_shot::Command settle(lob_shot::Manager & m)
{
  lob_shot::Command c;
  for (int i = 0; i < 60; ++i) {
    c = m.tick(0.02);
    if (m.state() == lob_shot::State::READY) {
      if (m.fire_latched()) c = m.tick(0.02);
      return c;
    }
    if (m.state() == lob_shot::State::INHIBITED) return c;
    if (m.status().rfind("IDLE (", 0) == 0) return c;
  }
  return c;
}
}  // namespace

int main()
{
  lob_shot::Config cfg;
  cfg.module_unlocked = true;
  cfg.module_docked = true;
  lob_shot::Manager mgr(cfg);
  mgr.request_trigger();
  auto cmd = settle(mgr);
  if (mgr.state() != lob_shot::State::READY) return fail("READY");
  if (!near(mgr.last_pitch_deg(), 33.27, 0.05)) return fail("pitch");
  if (lob_shot::to_z(cmd) != 1) return fail("z");

  // CRC 交叉：encode_a5 尾两字节 == tools::get_crc16(前11字节)
  auto a5 = lob_shot::encode_a5(static_cast<float>(cmd.pitch), static_cast<float>(cmd.yaw), 1);
  if (a5.size() != 13) return fail("a5 len");
  const uint16_t expect = tools::get_crc16(a5.data(), 11);
  const uint16_t got = static_cast<uint16_t>(a5[11] | (a5[12] << 8));
  if (got != expect) return fail("a5 crc != tools::get_crc16");
  if (!tools::check_crc16(a5.data(), 13)) return fail("check_crc16");

  // 同济 SP 帧 CRC
  auto sp = lob_shot::to_gimbal_sp(cmd);
  if (sp.mode != 1) return fail("sp mode");
  if (sp.head[0] != 'S' || sp.head[1] != 'P') return fail("sp head");
  uint16_t sp_crc = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&sp), sizeof(sp) - sizeof(sp.crc16));
  if (sp.crc16 != sp_crc) return fail("sp crc");

  cfg.base_yaw = M_PI / 2;
  lob_shot::Manager my(cfg);
  my.request_trigger();
  settle(my);
  if (near(my.last_yaw_deg(), mgr.last_yaw_deg(), 1.0)) return fail("base_yaw");

  cfg.base_yaw = 0;
  lob_shot::Manager mg(cfg);
  mg.request_trigger();
  mg.set_fire(true);
  settle(mg);
  mg.set_gate(true, false, false);
  settle(mg);
  if (mg.fire_latched()) return fail("fire clear");
  mg.set_gate(true, true, false);
  auto c2 = settle(mg);
  if (c2.control && lob_shot::to_z(c2) == 2) return fail("auto fire");

  cfg.windmill_radius = 30.0;
  lob_shot::Manager mw(cfg);
  mw.request_trigger();
  if (settle(mw).control) return fail("windmill");

  cfg.windmill_radius = 1.7;
  cfg.pitch_max_deg = 10.0;
  lob_shot::Manager mmax(cfg);
  mmax.request_trigger();
  if (settle(mmax).control) return fail("pitch_max");

  cfg.pitch_max_deg = 42.0;
  lob_shot::Manager mf(cfg);
  mf.request_trigger();
  mf.set_fire(true);
  if (lob_shot::to_z(settle(mf)) != 2) return fail("fire z");

  std::cout << "heavy_lob_offline_test PASS pitch=" << mgr.last_pitch_deg()
            << " a5_crc_ok gimbal_sp_crc_ok\n";
  return 0;
}
