#include <pty.h>
#include <poll.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>
#include <cstring>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "io/hero/config.hpp"
#include "io/hero/hero_serial.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_outpost/shooter.hpp"
#include "tools/math_tools.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/trajectory.hpp"

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}
std::vector<uint8_t> feedback_packet(float pitch, float yaw, float speed, uint8_t flags = 1)
{
  std::vector<uint8_t> packet(16);
  packet[0] = 0x5a;
  packet[1] = flags;
  std::memcpy(packet.data() + 2, &pitch, 4);
  std::memcpy(packet.data() + 6, &yaw, 4);
  std::memcpy(packet.data() + 10, &speed, 4);
  const auto crc = hero::crc16(packet.data(), 14);
  packet[14] = crc & 0xff;
  packet[15] = crc >> 8;
  return packet;
}
void protocol()
{
  const std::string check = "123456789";
  require(hero::crc16(reinterpret_cast<const uint8_t *>(check.data()), check.size()) == 0x6f91, "CRC reference vector");
  const auto encoded = hero::encode({true, true, -0.25, 0.125});
  require(encoded.size() == 13 && encoded[0] == 0xa5 && encoded[9] == 2 && encoded[10] == 0, "TX wire layout");
  require(encoded[1] == 0 && encoded[2] == 0 && encoded[3] == 0 && encoded[4] == 0x3e &&
          encoded[5] == 0 && encoded[6] == 0 && encoded[7] == 0x80 && encoded[8] == 0xbe, "TX angle sign and order");
  require(encoded[11] == 0x92 && encoded[12] == 0x5f, "TX fixed CRC bytes");
  require(hero::encode({false, true, 1, 2})[9] == 0, "Shoot requires control");
  require(hero::encode({true, true, 0, std::numeric_limits<double>::quiet_NaN()})[9] == 0, "Nonfinite command disabled");
  auto packet = feedback_packet(0.1f, -0.2f, 15.8f, 0x7b);
  std::vector<uint8_t> buffer{0x11, 0x23};
  buffer.insert(buffer.end(), packet.begin(), packet.begin() + 7);
  require(hero::parse(buffer).empty(), "Partial frame retained");
  buffer.insert(buffer.end(), packet.begin() + 7, packet.end());
  const auto decoded = hero::parse(buffer);
  require(decoded.size() == 1 && buffer.empty(), "Partial frame completed");
  require(decoded[0].enemy_color == 1 && decoded[0].game_status == 5 && decoded[0].reset_tracker &&
          decoded[0].camera_status && decoded[0].outpost_status, "RX flags");
  require(std::abs(decoded[0].pitch - 0.1) < 1e-6 && std::abs(decoded[0].yaw + 0.2) < 1e-6, "RX float layout");
  auto bad = packet;
  bad[5] ^= 0xff;
  buffer = bad;
  buffer.insert(buffer.end(), packet.begin(), packet.end());
  buffer.insert(buffer.end(), packet.begin(), packet.end());
  require(hero::parse(buffer).size() == 2 && buffer.empty(), "CRC resynchronization and concatenated frames");
}
void history()
{
  hero::FeedbackHistory history(15.8, 0.15);
  require(!history.latest(), "No startup feedback");
  auto stamp = hero::Clock::now();
  hero::Feedback feedback;
  feedback.bullet_speed = 15.6;
  feedback.yaw = 3.13;
  history.accept(feedback, stamp);
  require(std::abs(history.latest()->bullet_speed - 15.6) < 1e-5, "First valid speed initializes EMA");
  feedback.bullet_speed = 16;
  feedback.yaw = -3.13;
  history.accept(feedback, stamp + std::chrono::milliseconds(10));
  require(std::abs(history.latest()->bullet_speed - 15.66) < 1e-5, "EMA per packet");
  const auto orientation = history.imu_at(stamp + std::chrono::milliseconds(5), 0.1);
  require(orientation && std::abs(orientation->toRotationMatrix()(0, 0) + 1) < 0.001, "Yaw interpolation crosses pi correctly");
  require(!history.imu_at(stamp + std::chrono::seconds(1), 0.1), "Stale attitude rejected");
  auto epoch = history.latest()->epoch;
  feedback.reset_tracker = true;
  feedback.bullet_speed = std::numeric_limits<float>::quiet_NaN();
  history.accept(feedback, stamp + std::chrono::milliseconds(20));
  require(history.latest()->epoch == epoch + 1, "Reset advances epoch");
  require(std::abs(history.latest()->bullet_speed - 15.66) < 1e-5, "Invalid speed ignored");
  history.accept(feedback, stamp + std::chrono::milliseconds(30));
  require(history.latest()->epoch == epoch + 1, "Held reset does not continually restart");
  feedback.enemy_color = 0;
  history.accept(feedback, stamp + std::chrono::milliseconds(40));
  require(history.latest()->epoch == epoch + 2, "Color change advances epoch");
  const auto facing = hero::attitude(0, 0.2) * Eigen::Vector3d::UnitX();
  require(facing.z() > 0, "Hero positive pitch points up");
}
void layers_and_ballistics()
{
  std::vector<Eigen::Vector4d> armors{{5, 0, 0.1, 0}, {5, 0, 0.3, 0}, {5, 0, 0.2, 0}};
  require(auto_outpost::select_layer(armors, 0, 0.04) == 1, "Highest valid layer");
  armors = {{5, 0, 0.2, 0}, {5, 0, 0.23, 0}, {5, 0, 0, 0}};
  require(auto_outpost::select_layer(armors, std::atan2(0.2, 5), 0.04) == 0, "Close layers choose current pitch");
  armors[1][2] = 0.25;
  require(auto_outpost::select_layer(armors, std::atan2(0.2, 5), 0.04) == 1, "Separated layers choose highest");
  armors[1][2] = 0.24;
  require(auto_outpost::select_layer(armors, std::atan2(0.2, 5), 0.04) == 1, "Threshold equality chooses highest");
  armors = {{5, 0, 0, 0}, {5, 0, std::numeric_limits<double>::quiet_NaN(), 0}};
  require(auto_outpost::select_layer(armors, 0, 0.04) == -1, "No valid layers");
  armors.push_back({5, 0, 0.1, 0});
  require(auto_outpost::select_layer(armors, 0, 0.04) == 2, "One valid layer");
  tools::Trajectory ideal(15.8, 5, 0.2, 0);
  tools::Trajectory drag(15.8, 5, 0.2, 0.0068);
  const double expected = std::atan((15.8 * 15.8 - std::sqrt(std::pow(15.8, 4) - 9.7915 * (9.7915 * 25 + 0.4 * 15.8 * 15.8))) / (9.7915 * 5));
  require(!ideal.unsolvable && std::abs(ideal.pitch - expected) < 1e-5, "RK4 matches analytic vacuum trajectory");
  require(!drag.unsolvable && drag.pitch > ideal.pitch && drag.fly_time > ideal.fly_time, "Quadratic drag participates");
  require(tools::Trajectory(0, 5, 0).unsolvable && tools::Trajectory(15.8, 500, 0).unsolvable, "Invalid and unreachable trajectories");
}
void serial_transport(const YAML::Node & config)
{
  int master = -1, slave = -1;
  char path[256]{};
  require(openpty(&master, &slave, path, nullptr, nullptr) == 0, "Create pseudo serial port");
  termios settings{};
  require(tcgetattr(slave, &settings) == 0, "Read PTY settings");
  cfmakeraw(&settings);
  require(tcsetattr(slave, TCSANOW, &settings) == 0, "Disable PTY echo before receiver starts");
  auto serial_config = YAML::Clone(config["serial"]);
  serial_config["device"] = std::string(path);
  serial_config["timestamp_offset"] = 0;
  {
    hero::Serial serial(serial_config);
    const auto packet = feedback_packet(0.1, -0.2, 15.8);
    const auto deadline = hero::Clock::now() + std::chrono::seconds(2);
    while (!serial.state() && hero::Clock::now() < deadline) {
      require(write(master, packet.data(), packet.size()) == static_cast<ssize_t>(packet.size()), "Write feedback");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto sample = serial.state();
    require(sample.has_value(), "PTY feedback received");
    require(serial.send({true, true, 0.2, 0.1}, sample->epoch), "PTY TX complete");
    pollfd event{master, POLLIN, 0};
    require(poll(&event, 1, 1000) > 0, "PTY TX readable");
    uint8_t bytes[64]{};
    const auto count = read(master, bytes, sizeof(bytes));
    require(count == 13 && bytes[0] == 0xa5 && bytes[9] == 2, "PTY hero wire command");
    auto stats = serial.stats();
    require(stats.connected && stats.rx_packets > 0 && stats.tx_packets == 1 &&
            stats.tx_fire_packets == 1 && stats.last_status == 2, "TX diagnostics count actual writes");
    require(serial.send({true, true, 0.2, 0.1}, sample->epoch + 1), "Wrong epoch yields disabled packet");
    require(poll(&event, 1, 1000) > 0, "Epoch rejection readable");
    require(read(master, bytes, sizeof(bytes)) == 13 && bytes[9] == 0, "Old generation cannot shoot");
    std::this_thread::sleep_for(std::chrono::milliseconds(130));
    require(serial.send({true, true, 0.2, 0.1}, sample->epoch), "Stale TX disabled packet");
    require(poll(&event, 1, 1000) > 0, "Stale packet readable");
    require(read(master, bytes, sizeof(bytes)) == 13 && bytes[9] == 0, "Stale feedback suppresses fire");
    stats = serial.stats();
    require(stats.tx_packets == 3 && stats.tx_fire_packets == 1 && stats.tx_suppressed == 2 &&
            stats.last_status == 0, "Blocked requests do not count as fire writes");
  }
  close(master);
  close(slave);
}

auto_aim::Armor observation(auto_aim::ArmorName name, double angle = 0)
{
  auto_aim::Armor armor(0, 6, 0.99f, {0, 0, 100, 40}, {{0, 0}, {100, 0}, {100, 40}, {0, 40}});
  armor.name = name;
  armor.type = auto_aim::small;
  const double radius = name == auto_aim::outpost ? 0.2765 : 0.2;
  armor.xyz_in_world = {5 - radius * std::cos(angle), -radius * std::sin(angle), 0.5};
  armor.ypr_in_world = {angle, 0, 0};
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  return armor;
}

void tracking_continuity()
{
  const Eigen::VectorXd covariance = Eigen::VectorXd::Constant(11, 0.001);
  auto_aim::Target target(4, 0.2, covariance, 0, 2);
  auto armor = observation(auto_aim::three);
  auto stamp = hero::Clock::now();
  target.update(armor, stamp);
  target.update(armor, stamp += std::chrono::milliseconds(5));
  require(target.state_str() == "TRACKING", "Synthetic target acquired");
  for (int cycle = 0; cycle < 5; ++cycle) {
    target.update(nullptr, stamp += std::chrono::milliseconds(5));
    target.update(nullptr, stamp += std::chrono::milliseconds(5));
    require(target.state(), "Two consecutive misses remain temporary");
    target.update(armor, stamp += std::chrono::milliseconds(5));
    require(target.state_str() == "TRACKING", "Reacquisition resets consecutive misses");
  }
  for (int index = 0; index < 3; ++index)
    target.update(nullptr, stamp += std::chrono::milliseconds(5));
  require(!target.state(), "Three consecutive misses exceed limit");
}

// 系统级回归：前哨站先正转再反转，最终 w 的符号必须跟着翻过来。
// 注意：这个测试【不能】单独证明 Target::monitor_outpost_direction 起作用——关掉监测它也会通过，
// 因为合成数据下反转会让滤波器发散、丢失，重新加锁(set_w)同样能得到正确符号。
// 监测本身目前只靠实车日志里的 "outpost direction corrected" 验证。
// 合成数据模拟真实情况：三块板相隔 120°，只有朝向相机 ±0.7 rad 内的板可见，板之间有短暂空档。
void outpost_direction_correction()
{
  // 和 tracker.cpp 里前哨站的初始协方差一致：w 的方差大，加锁前几帧能从观测里学出转向，之后由 set_w 固定
  Eigen::VectorXd covariance{{1, 64, 1, 64, 1, 9, 0.4, 10, 0.0001, 0, 0}};
  auto_aim::Target target(3, 0.2765, covariance, 5, 120, 0, 0, 0);
  auto stamp = hero::Clock::now();
  double phase = 0;
  const double rate = 2.5, step = 0.01;
  auto feed = [&](double direction, double seconds) {
    for (double t = 0; t < seconds; t += step) {
      phase += direction * rate * step;
      stamp += std::chrono::duration_cast<hero::Clock::duration>(std::chrono::duration<double>(step));
      const auto_aim::Armor * visible = nullptr;
      auto_aim::Armor armor = observation(auto_aim::outpost);
      for (int plate = 0; plate < 3; ++plate) {
        const double angle = tools::limit_rad(phase + plate * 2 * CV_PI / 3);
        if (std::abs(angle) < 0.7) { armor = observation(auto_aim::outpost, angle); visible = &armor; }
      }
      if (target.state()) target.predict(stamp);  // 与 Tracker::update_targets 一致：先预测、再更新
      target.update(visible, stamp);
    }
  };
  feed(+1, 3.0);
  require(target.state(), "Outpost stays tracked while spinning forward");
  require(target.ekf_x()[7] > 1.0, "Forward spin gives positive w");
  feed(-1, 4.0);
  require(target.state(), "Outpost stays tracked after the spin reverses");
  require(target.ekf_x()[7] < -1.0, "Reversed spin flips the sign of w");
}

void repeated_outpost_window(const std::string & config_path, const YAML::Node & config)
{
  const double speed = 15.8;
  const double angular_speed = 2.5;
  tools::Trajectory trajectory(speed, 5 - 0.2765, 0.5, config["k_air_resistance"].as<double>());
  require(!trajectory.unsolvable, "Synthetic outpost trajectory");
  const double initial_angle = -angular_speed * (config["shoot_delay"].as<double>() + trajectory.fly_time) - 0.005;
  const auto armor = observation(auto_aim::outpost, initial_angle);
  const Eigen::VectorXd covariance = Eigen::VectorXd::Constant(11, 0.001);
  const auto start = hero::Clock::now();
  auto_aim::Target target(armor, start, 0.2765, 3, covariance, 0, 0, angular_speed);
  auto_outpost::Shooter shooter(config_path);
  const Eigen::Vector3d ypr{0, -trajectory.pitch, 0};
  const auto command = shooter.shoot({target}, start, speed, false, ypr);
  require(command.control && command.shoot, "Highest layer enters first fire window");
  const auto same_window = shooter.shoot({target}, start + std::chrono::milliseconds(1), speed, false, ypr);
  require(same_window.control && !same_window.shoot, "One fire pulse per window, no double fire in the same pass");
  const auto outside = shooter.shoot({target}, start + std::chrono::milliseconds(10), speed, false, ypr);
  require(outside.control && !outside.shoot, "Fire clears immediately outside phase window");
  const auto period = std::chrono::duration_cast<hero::Clock::duration>(std::chrono::duration<double>(2 * CV_PI / angular_speed));
  require(shooter.shoot({target}, start + period, speed, false, ypr).shoot,
          "Same highest armor can fire on its next revolution");
  require(!shooter.shoot({}, start + period, speed, false, ypr).control, "No target clears command");
}
}
int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "Expected configuration path");
    const auto config = hero::validate_config(argv[1]);
    protocol();
    history();
    layers_and_ballistics();
    serial_transport(config);
    tracking_continuity();
    outpost_direction_correction();
    repeated_outpost_window(argv[1], config);
    tools::ThreadSafeQueue<int> queue(2);
    auto waiter = std::async(std::launch::async, [&] {
      try { queue.pop(); return false; } catch (const std::runtime_error &) { return true; }
    });
    queue.close();
    require(waiter.wait_for(std::chrono::seconds(1)) == std::future_status::ready && waiter.get(), "Queue close wakes consumer");
    std::cout << "PASS protocol, feedback, pose, layers, ballistics, serial transport, tracking, outpost direction, repeated fire windows, queue shutdown\n";
    return 0;
  } catch (const std::exception & error) { std::cerr << error.what() << '\n'; return 1; }
}
