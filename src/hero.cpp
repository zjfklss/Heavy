#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include <atomic>
#include <csignal>
#include <condition_variable>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include "io/hero/config.hpp"
#include "io/camera.hpp"
#include "io/hero/hero_serial.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_outpost/shooter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace
{
volatile std::sig_atomic_t interrupted = 0;
class ConfigFile
{
public:
  explicit ConfigFile(const YAML::Node & config)
  {
    char name[] = "/tmp/hero-config-XXXXXX";
    const int descriptor = mkstemp(name);
    if (descriptor < 0) throw std::runtime_error("Cannot create resolved configuration");
    close(descriptor);
    path = name;
    std::ofstream output(path);
    output << YAML::Dump(config);
    if (!output) throw std::runtime_error("Cannot write resolved configuration");
  }
  ~ConfigFile() { std::filesystem::remove(path); }
  std::string path;
};
struct DecisionInput
{
  std::list<auto_aim::Target> targets;
  hero::Clock::time_point stamp;
  hero::Clock::time_point received;
  double speed;
  Eigen::Vector3d ypr;
  uint64_t epoch;
  bool feedback_valid;
};
struct DecisionOutput
{
  io::Command command{false, false, 0, 0};
  auto_outpost::AimPoint aim{false, Eigen::Vector4d::Zero()};
  uint64_t epoch = 0;
};
class Decisions
{
public:
  Decisions(std::string config, hero::Serial * serial, bool realtime, double timeout, bool no_fire)
  : config_(std::move(config)), serial_(serial), realtime_(realtime), timeout_(timeout), no_fire_(no_fire)
  {
    thread_ = std::thread([this] { run(); });
  }
  ~Decisions()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    ready_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (serial_) serial_->send({false, false, 0, 0}, 0);
  }
  void push(DecisionInput input)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = std::move(input);
      ++generation_;
    }
    ready_.notify_all();
  }
  DecisionOutput snapshot()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_;
  }
  void finish_frame()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ready_.wait(lock, [&] { return failure_ || completed_ >= generation_; });
    if (failure_) std::rethrow_exception(failure_);
  }
private:
  void run()
  {
    try {
      tools::Plotter plotter;
      std::unique_ptr<auto_outpost::Shooter> shooter;
      uint64_t epoch = 0;
      auto last_report = hero::Clock::now();
      uint64_t fire_requests = 0;
      while (true) {
        std::optional<DecisionInput> input;
        uint64_t generation;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          ready_.wait_for(lock, std::chrono::milliseconds(1), [&] {
            return stop_ || generation_ != completed_;
          });
          if (stop_) break;
          if (!latest_) continue;
          input = latest_;
          generation = generation_;
          if (!realtime_ && completed_ == generation_) continue;
        }
        if (!shooter || epoch != input->epoch) {
          shooter = std::make_unique<auto_outpost::Shooter>(config_);
          epoch = input->epoch;
        }
        DecisionOutput output;
        output.epoch = input->epoch;
        const bool expired = realtime_ && (
          std::chrono::duration<double>(hero::Clock::now() - input->stamp).count() > timeout_ ||
          std::chrono::duration<double>(hero::Clock::now() - input->received).count() > timeout_);
        if (!expired) {
          output.command = shooter->shoot(input->targets, input->stamp, input->speed, realtime_, input->ypr);
          output.aim = shooter->debug_aim_point_;
        }
        const int requested_control = hero::encode(output.command)[9];
        if (output.command.shoot) ++fire_requests;
        if (no_fire_) output.command.shoot = false;
        auto gated_command = input->feedback_valid ? output.command : io::Command{false, false, 0, 0};
        bool tx_ok = false;
        if (serial_) tx_ok = serial_->send(gated_command, input->epoch);
        const auto packet = hero::encode(output.command);
        const auto stats = serial_ ? serial_->stats() : hero::TransportStats{};
        plotter.plot({{"command_yaw", output.command.yaw * 180 / CV_PI},
                      {"command_pitch", output.command.pitch * 180 / CV_PI},
                      {"control", packet[9]}, {"requested_control", requested_control},
                      {"gated_control", hero::encode(gated_command)[9]},
                      {"feedback_valid", input->feedback_valid}, {"decision_expired", expired},
                      {"tx_ok", tx_ok}, {"tx_control", tx_ok ? stats.last_status : -1},
                      {"rx_packets", stats.rx_packets}, {"tx_fire_packets", stats.tx_fire_packets}});
        if (hero::Clock::now() - last_report >= std::chrono::seconds(1)) {
          tools::logger()->info(
            "[Decision] requested={} gated={} tx={} feedback_valid={} expired={} no_fire={} fire_requests={} rx={} tx_packets={} tx_fire={} tx_failures={} tx_suppressed={}",
            requested_control, hero::encode(gated_command)[9], tx_ok ? stats.last_status : -1,
            input->feedback_valid, expired, no_fire_, fire_requests, stats.rx_packets, stats.tx_packets,
            stats.tx_fire_packets, stats.tx_failures, stats.tx_suppressed);
          last_report = hero::Clock::now();
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          output_ = output;
          completed_ = generation;
        }
        ready_.notify_all();
      }
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        failure_ = std::current_exception();
      }
      if (serial_) serial_->send({false, false, 0, 0}, 0);
      ready_.notify_all();
    }
  }
  std::string config_;
  hero::Serial * serial_;
  bool realtime_;
  double timeout_;
  bool no_fire_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::optional<DecisionInput> latest_;
  DecisionOutput output_;
  uint64_t generation_ = 0;
  uint64_t completed_ = 0;
  bool stop_ = false;
  std::exception_ptr failure_;
  std::thread thread_;
};
}

int main(int argc, char ** argv)
{
  const std::string keys =
    "{help h usage ? | | Show usage}"
    "{@config | configs/hero.yaml | Hero configuration}"
    "{check-config | false | Validate configuration without models or peripherals}"
    "{smoke-test | false | Process three blank frames without peripherals}"
    "{camera-test | false | Measure camera acquisition FPS without detection or serial}"
    "{video | | Offline video, no camera or serial}"
    "{yaw | 0 | Offline fixed hero yaw, radians}"
    "{pitch | 0 | Offline fixed hero pitch, positive up, radians}"
    "{frames | 0 | Frame limit, zero means unlimited}"
    "{show | false | Show reprojection window}"
    "{no-fire | false | Follow targets while suppressing fire commands}"
    "{no-serial | false | Live camera with fixed yaw/pitch; never open serial}";
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) { cli.printMessage(); return 0; }
  try {
    auto config = hero::validate_config(cli.get<std::string>(0));
    const bool smoke = cli.get<bool>("smoke-test");
    const bool camera_test = cli.get<bool>("camera-test");
    const bool show = cli.get<bool>("show");
    const auto video_path = cli.get<std::string>("video");
    const double yaw = cli.get<double>("yaw"), pitch = cli.get<double>("pitch");
    int frame_limit = cli.get<int>("frames");
    if (!cli.check()) { cli.printErrors(); return 2; }
    if (frame_limit < 0 || !std::isfinite(yaw) || !std::isfinite(pitch))
      throw std::runtime_error("Invalid frames/yaw/pitch");
    if ((smoke && !video_path.empty()) || (camera_test && (smoke || !video_path.empty())))
      throw std::runtime_error("Choose only one input source");
    if (cli.get<bool>("check-config")) {
      std::cout << "Valid hero configuration; RX=16 TX=13, no peripherals opened\n";
      return 0;
    }
    ConfigFile resolved(config);
    std::signal(SIGINT, [](int) { interrupted = 1; });
    std::signal(SIGTERM, [](int) { interrupted = 1; });
    if (camera_test) {
      io::Camera camera(resolved.path);
      if (!frame_limit) frame_limit = 1000;
      int frames = 0;
      auto first = hero::Clock::time_point{};
      auto last = hero::Clock::now();
      while (!interrupted && frames < frame_limit) {
        cv::Mat image;
        hero::Clock::time_point stamp;
        camera.read(image, stamp);
        if (image.empty()) {
          if (hero::Clock::now() - last > std::chrono::seconds(5))
            throw std::runtime_error("No camera frames for five seconds");
          continue;
        }
        last = stamp;
        if (frames++ == 0) first = stamp;
        if (show) { cv::imshow("reprojection", image); if (cv::waitKey(1) == 27) break; }
      }
      const double elapsed = std::chrono::duration<double>(last - first).count();
      std::cout << "camera_frames=" << frames << " acquisition_fps="
                << (elapsed > 0 ? (frames - 1) / elapsed : 0) << '\n';
      return 0;
    }
    const bool offline = smoke || !video_path.empty();
    if (smoke) frame_limit = 3;
    tools::logger();
    auto_aim::multithread::MultiThreadDetector detector(resolved.path, false);
    auto_aim::Solver solver(resolved.path);
    auto tracker = std::make_unique<auto_aim::Tracker>(resolved.path, solver);
    std::unique_ptr<hero::Serial> serial;
    std::unique_ptr<io::Camera> camera;
    cv::VideoCapture video;
    if (!offline) {
      camera = std::make_unique<io::Camera>(resolved.path);
      if (!cli.get<bool>("no-serial")) serial = std::make_unique<hero::Serial>(config["serial"]);
      else tools::logger()->info("[Input] Live camera, serial disabled, fixed yaw={} pitch={} rad", yaw, pitch);
    } else if (!smoke && !video.open(video_path)) throw std::runtime_error("Cannot open video: " + video_path);
    Decisions decisions(resolved.path, serial.get(), !offline, config["decision_timeout"].as<double>(), cli.get<bool>("no-fire"));
    double fps = smoke ? 30 : video.get(cv::CAP_PROP_FPS);
    if (!std::isfinite(fps) || fps <= 0) fps = 30;
    std::atomic<bool> stop{false};
    std::atomic<bool> source_done{false};
    std::atomic<int> pending{0};
    std::exception_ptr producer_failure;
    const auto begin = hero::Clock::now();
    std::thread producer([&] {
      try {
        int count = 0;
        while (!stop && !interrupted && (!frame_limit || count < frame_limit)) {
          if (pending >= 2) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
          cv::Mat image;
          auto stamp = hero::Clock::now();
          if (camera) camera->read(image, stamp);
          else if (smoke) image = cv::Mat::zeros(config["image_height"].as<int>(), config["image_width"].as<int>(), CV_8UC3);
          else if (!video.read(image)) break;
          if (image.empty()) continue;
          if (offline) stamp = begin + std::chrono::duration_cast<hero::Clock::duration>(std::chrono::duration<double>(count / fps));
          ++pending;
          detector.push(image, stamp);
          ++count;
        }
      } catch (...) { producer_failure = std::current_exception(); }
      detector.close();
      source_done = true;
    });
    std::exception_ptr loop_failure;
    int frames = 0;
    uint64_t epoch = 0;
    bool feedback_was_valid = false;
    tools::Plotter plotter;
    auto last_report = begin;
    int previous_frames = 0;
    uint64_t previous_captured = 0;
    try {
      while (!interrupted) {
        auto frame = detector.debug_pop_for(std::chrono::milliseconds(100));
        if (!frame) {
          if (source_done) break;
          continue;
        }
        auto [image, armors, stamp] = std::move(*frame);
        --pending;
        const auto now = hero::Clock::now();
        auto orientation = hero::attitude(yaw, pitch);
        const int fallback_color = config["enemy_color"].as<std::string>() == "blue" ? 1 : 0;
        int color = fallback_color;
        double speed = config["serial"]["bullet_speed"].as<double>();
        uint64_t next_epoch = 1;
        bool feedback_valid = offline;
        if (serial) {
          const auto sample = serial->state();
          const auto aligned = serial->imu_at(stamp - std::chrono::milliseconds(1));
          if (sample) {
            color = sample->feedback.enemy_color;
            speed = sample->bullet_speed;
            next_epoch = sample->epoch;
            feedback_valid = aligned && std::chrono::duration<double>(now - sample->stamp).count() <= config["serial"]["timeout"].as<double>();
          }
          if (aligned) orientation = *aligned;
        }
        if (epoch != next_epoch || feedback_was_valid != feedback_valid) {
          tools::logger()->info("[Feedback] epoch={} valid={} previous_epoch={} previous_valid={}",
                                next_epoch, feedback_valid, epoch, feedback_was_valid);
          tracker = std::make_unique<auto_aim::Tracker>(resolved.path, solver);
          epoch = next_epoch;
          feedback_was_valid = feedback_valid;
        }
        tracker->set_enemy_color(color == 0 ? auto_aim::red : auto_aim::blue);
        solver.set_R_gimbal2world(orientation);
        const Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
        auto targets = tracker->track(armors, stamp, ypr[0], true, io::Mode::auto_aim);
        decisions.push({targets, stamp, now, speed, ypr, epoch, feedback_valid});
        decisions.finish_frame();
        const auto output = decisions.snapshot();
        ++frames;
        if (offline) {
          const auto packet = hero::encode(output.command);
          std::cout << "frame=" << frames << " targets=" << targets.size()
                    << " control=" << int(packet[9]) << " tx_bytes=" << packet.size() << '\n';
        }
        const double elapsed = std::chrono::duration<double>(now - begin).count();
        plotter.plot({{"fps", elapsed > 0 ? frames / elapsed : 0}, {"bullet_speed", speed},
                      {"gimbal_yaw", ypr[0] * 180 / CV_PI}, {"gimbal_pitch", -ypr[1] * 180 / CV_PI}});
        if (now - last_report >= std::chrono::seconds(1)) {
          const double interval = std::chrono::duration<double>(now - last_report).count();
          const auto captured = camera ? camera->frames_captured() : 0;
          const double detection_fps = (frames - previous_frames) / interval;
          const double acquisition_fps = camera ? (captured - previous_captured) / interval : 0;
          tools::logger()->info(
            "[Vision] detection_fps={:.1f} acquisition_fps={:.1f} image_age_ms={:.1f} armors={} targets={} tracker={} feedback_valid={}",
            detection_fps, acquisition_fps, std::chrono::duration<double, std::milli>(now - stamp).count(),
            armors.size(), targets.size(), tracker->state_str(), feedback_valid);
          previous_frames = frames;
          previous_captured = captured;
          last_report = now;
        }
        if (show) {
          if (tracker->state()) {
            const auto target = tracker->get_target();
            for (const auto & armor : target.armor_xyza_list())
              tools::draw_points(image, solver.reproject_armor(armor.head<3>(), armor[3], target.armor_type, target.name), {0, 255, 0});
            if (output.aim.valid)
              tools::draw_points(image, solver.reproject_armor(output.aim.xyza.head<3>(), output.aim.xyza[3], target.armor_type, target.name), {0, 0, 255});
          }
          tools::draw_text(image, "yaw=" + std::to_string(ypr[0]) + " pitch=" + std::to_string(-ypr[1]), {10, 30}, {255, 255, 255});
          cv::resize(image, image, {}, 0.5, 0.5);
          cv::imshow("reprojection", image);
          const auto key = cv::waitKey(1);
          if (key == 27 || key == 'q') break;
        }
      }
    } catch (...) { loop_failure = std::current_exception(); }
    stop = true;
    detector.close();
    if (producer.joinable()) producer.join();
    if (producer_failure) std::rethrow_exception(producer_failure);
    if (loop_failure) {
      try { std::rethrow_exception(loop_failure); }
      catch (const std::runtime_error & error) {
        if (std::string(error.what()) != "Queue closed") throw;
      }
    }
    std::cout << "processed_frames=" << frames << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "[heavy_hero] " << error.what() << '\n';
    return 1;
  }
}
