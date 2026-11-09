#include "bridge.hpp"

#include <unistd.h>

#include <cmath>
#include <fstream>

#include "io/hero/config.hpp"

namespace io::simulation
{
namespace
{
using nlohmann::json;
void require(bool value, const char * message)
{
  if (!value) throw std::invalid_argument(message);
}
Eigen::Vector3d vector(const json & v)
{
  require(v.is_array() && v.size() == 3, "Invalid simulation vector");
  Eigen::Vector3d result(v[0].get<double>(), v[1].get<double>(), v[2].get<double>());
  require(result.allFinite(), "Non-finite simulation vector");
  return result;
}
Eigen::Matrix3d rotation(const json & columns)
{
  require(columns.is_array() && columns.size() == 3, "Invalid simulation rotation");
  Eigen::Matrix3d r;
  for (int c = 0; c < 3; ++c) r.col(c) = vector(columns[c]);
  require(
    (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-6 && std::abs(r.determinant() - 1) < 1e-6,
    "Invalid simulation rotation");
  return r;
}
Angles angles(const Eigen::Vector3d & direction)
{
  return {std::atan2(direction.y(), direction.x()), std::atan2(direction.z(), std::hypot(direction.x(), direction.y()))};
}
}  // namespace

Angles to_world(const Eigen::Matrix3d & base, double yaw, double pitch)
{
  return angles(base * (hero::attitude(yaw, pitch) * Eigen::Vector3d::UnitX()));
}
Angles to_simulator(const Eigen::Matrix3d & base, double yaw, double pitch)
{
  return angles(base.transpose() * (hero::attitude(yaw, pitch) * Eigen::Vector3d::UnitX()));
}

YAML::Node load_base_config(const std::string & path) { return hero::validate_config(path); }

Bridge::Bridge(rm_sim::adapters::ServerOptions options) : server_(std::move(options)) {}
Bridge::~Bridge()
{
  if (!temporary_.empty()) {
    std::error_code error;
    std::filesystem::remove_all(temporary_, error);
  }
}
hero::Clock::time_point Bridge::stamp(double seconds) const
{
  return epoch_ + std::chrono::duration_cast<hero::Clock::duration>(std::chrono::duration<double>(seconds));
}
void Bridge::accept(const std::string & base_config)
{
  server_.accept();
  const auto & hello = server_.hello();
  session_ = hello.at("session_id");
  const auto & c = hello.at("configuration");
  realtime_ = c.value("realtime", false);
  epoch_ = realtime_ ? hero::Clock::now() : hero::Clock::time_point{};
  const auto & a = c.at("actuator");
  const auto & camera = c.at("camera");
  const auto & k = camera.at("calibration");
  // The hero Solver assumes coincident joint and muzzle origins. Reject unsupported geometry explicitly.
  require(
    vector(a.at("yaw_offset")).norm() < 1e-9 && vector(a.at("pitch_offset")).norm() < 1e-9 &&
      vector(a.at("muzzle_mount").at("translation")).norm() < 1e-9 &&
      (rotation(a.at("muzzle_mount").at("rotation")) - Eigen::Matrix3d::Identity()).norm() < 1e-9,
    "Hero simulation requires zero joint offsets and an identity muzzle mount");
  base_rotation_ = rotation(a.at("base_pose").at("rotation"));
  bullet_speed_ = a.at("nominal_muzzle_speed");
  fire_interval_ = a.at("fire_interval");
  // auto_outpost::Shooter refuses speeds outside (10, 20) m/s; fail at handshake rather than never aiming.
  require(
    std::isfinite(bullet_speed_) && bullet_speed_ > 10 && bullet_speed_ < 20 && std::isfinite(fire_interval_) &&
      fire_interval_ >= 0,
    "Hero simulation requires a 42 mm nominal muzzle speed in (10, 20) m/s and a nonnegative fire interval");
  double hz = a.at("feedback_frequency");
  require(std::isfinite(hz) && hz > 0, "Invalid feedback frequency");
  feedback_age_ = std::max(.05, 2 / hz);
  wait_limit_ =
    std::max(camera.at("latency").get<double>(), a.at("feedback_latency").get<double>()) + 2 / hz + .05;
  auto config = load_base_config(base_config);
  config["camera_matrix"] = std::vector<double>{k.at("fx"), 0, k.at("cx"), 0, k.at("fy"), k.at("cy"), 0, 0, 1};
  config["distort_coeffs"] = std::vector<double>(5, 0);
  auto r = rotation(a.at("camera_mount").at("rotation"));
  std::vector<double> values;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) values.push_back(r(row, col));
  config["R_camera2gimbal"] = values;
  auto t = vector(a.at("camera_mount").at("translation"));
  config["t_camera2gimbal"] = std::vector<double>{t.x(), t.y(), t.z()};
  config["R_gimbal2imubody"] = std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1};
  config["image_width"] = k.at("width").get<int>();
  config["image_height"] = k.at("height").get<int>();
  // Rendered frames cover the whole sensor; a live-camera ROI would crop the wrong pixels.
  config["use_roi"] = false;
  const auto color = camera.value("enemy_color", config["enemy_color"].as<std::string>());
  require(color == "blue" || color == "red", "Invalid simulation enemy color");
  enemy_color_ = color == "red" ? 0 : 1;
  config["enemy_color"] = color;
  config["serial"]["bullet_speed"] = bullet_speed_;
  char directory[] = "/tmp/hero-simulation-XXXXXX";
  auto path = ::mkdtemp(directory);
  if (!path) throw std::runtime_error("Cannot create simulation calibration directory");
  temporary_ = path;
  config_path_ = (temporary_ / "profile.yaml").string();
  std::ofstream output(config_path_);
  output << YAML::Dump(config);
  output.close();
  if (!output) throw std::runtime_error("Cannot write simulation calibration");
}
std::optional<Step> Bridge::next()
{
  auto message = server_.next();
  if (!message) return std::nullopt;
  time_ = message->header.at("time");
  Step result;
  result.time = time_;
  for (const auto & f : message->header.at("feedback")) {
    hero::Feedback feedback;
    feedback.yaw = f.at("yaw").get<double>();
    feedback.pitch = f.at("pitch").get<double>();
    feedback.enemy_color = enemy_color_;
    feedback.bullet_speed = bullet_speed_;
    history_.accept(feedback, stamp(f.at("capture_time")));
    const auto world = to_world(base_rotation_, feedback.yaw, feedback.pitch);
    result.feedback.push_back({f.at("capture_time"), world.yaw, world.pitch});
    latest_feedback_ = f.at("capture_time");
    ++feedback_;
  }
  for (const auto & image : message->header.at("images")) {
    auto offset = image.at("offset").get<size_t>();
    cv::Mat view(
      image.at("height").get<int>(), image.at("width").get<int>(), CV_8UC3, message->payload.data() + offset,
      image.at("stride").get<size_t>());
    pending_.push_back({view.clone(), image.at("capture_time"), image.at("sequence")});
    queued_bytes_ += view.total() * view.elemSize();
    ++frames_;
  }
  while (!pending_.empty()) {
    const auto & p = pending_.front();
    const bool missing_feedback = latest_feedback_ + 1e-9 < p.capture;
    bool expired = queued_bytes_ > rm_sim::adapters::max_payload_bytes ||
                   (missing_feedback && time_ - p.capture > wait_limit_);
    if (missing_feedback && !expired) break;
    auto q = history_.imu_at(stamp(p.capture), feedback_age_);
    if (!expired && q)
      result.frames.push_back(
        {p.bgr, stamp(p.capture), Eigen::Quaterniond(base_rotation_ * q->toRotationMatrix()), p.sequence});
    else
      ++dropped_;
    queued_bytes_ -= p.bgr.total() * p.bgr.elemSize();
    pending_.pop_front();
  }
  return result;
}
void Bridge::reply(const std::optional<io::Command> & command, nlohmann::json telemetry)
{
  std::vector<rm_sim::adapters::Command> commands;
  if (command) {
    rm_sim::adapters::Command c;
    c.control = command->control && std::isfinite(command->yaw) && std::isfinite(command->pitch);
    if (c.control) {
      const auto local = to_simulator(base_rotation_, command->yaw, command->pitch);
      c.yaw = local.yaw;
      c.pitch = local.pitch;
      // The hero shoot flag is a level held while inside the fire window; on each new result map
      // eligible levels to unique single-shot requests at the simulator's nominal fire interval.
      c.fire = command->shoot && time_ + 1e-9 >= next_fire_;
      if (c.fire) {
        c.request_id = session_ + "-" + std::to_string(shots_++);
        next_fire_ = time_ + fire_interval_;
      }
    }
    last_command_ = c;
    commands.push_back(std::move(c));
  }
  telemetry["control"] = last_command_.control;
  telemetry["yaw"] = last_command_.yaw;
  telemetry["pitch"] = last_command_.pitch;
  telemetry["fire"] = !commands.empty() && commands.back().fire;
  telemetry["images_received"] = frames_;
  telemetry["feedback_received"] = feedback_;
  telemetry["dropped_images"] = dropped_;
  telemetry["pending_images"] = pending_.size();
  telemetry["shots_requested"] = shots_;
  server_.reply(commands, telemetry);
}
}  // namespace io::simulation
