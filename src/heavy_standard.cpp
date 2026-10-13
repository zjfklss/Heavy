// 上车主入口：同济 mt_standard 的线程形状 + 英雄电控 HeroBoard（0xA5 下发 / 0x5A 回传）。
//
// 线程：
//   采集线程  camera.read（或 --video 回放）→ 自瞄时 detector.push（异步推理），其它模式放进 latest
//   主线程    自瞄：detector.debug_pop_for → Tracker → Aimer → Shooter → board.send
//             打符：latest → Buff_Detector → Buff 目标/Aimer → board.send
//             吊射：不取图，lob_shot::Manager 20ms 一拍
//   调试线程  DebugSink：缩放/叠图/JPEG → Foxglove（/raw /vis /debug + 参数面板），可选 imshow/MCAP
//
// pitch 符号：同济世界系抬头为负，英雄 0xA5 抬头为正 → 自瞄/打符 send() 默认翻号；吊射本就抬头正，send(..., false)。
// 本轮只用短焦相机与内参；电控切长焦只打 warn。
// 调参手感对齐旧 ROS2 栈：改 yaml 保存即热加载，或在 Foxglove 参数面板直接改（见 docs/操作手速查.md）。
#include <atomic>
#include <chrono>
#include <cmath>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <fmt/ranges.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/hero_board.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/lob_shot/command.hpp"
#include "tasks/lob_shot/manager.hpp"
#include "tools/debug_sink.hpp"
#include "tools/exiter.hpp"
#include "tools/live_config.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"
#include "tools/replay.hpp"
#include "tools/thread_safe_queue.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | configs/heavy_standard.yaml | yaml }"
  "{mode           | | 覆盖 yaml 的 mode：auto_aim/outpost/small_buff/big_buff/lob/idle }"
  "{fake-board     | false | 不开串口（桌面调试，IMU 恒为单位阵，不下发）}"
  "{video          | | 离线回放 records/xxx（不带后缀，读 .avi+.txt），隐含 --fake-board }"
  "{video-fast     | false | 回放不按时间轴，压测推理吞吐（预测失真，只看 fps）}"
  "{record         | false | 录 records/<时间>.avi+.txt，之后可 --video 回放 }"
  "{imshow         | | 本机 raw/vis 窗口（Remmina 兜底），覆盖 debug.imshow }"
  "{plot p         | | PlotJuggler UDP 开关，覆盖 debug.plot（默认开）}"
  "{plot-host      | | PlotJuggler 目标 IP，覆盖 debug.plot_host（默认 127.0.0.1）}"
  "{plot-port      | | PlotJuggler UDP 端口，覆盖 debug.plot_port（默认 9870）}";

// 注：OpenCV 的 cli.has() 对有默认值的 key 恒为 true，所以「覆盖 yaml」的开关默认值必须留空

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace
{

// —— 热加载分组：改了哪组 key 就重建哪组对象；不在任何组里的 key 需重启 ——
const std::set<std::string> kSolverKeys = {
  "camera_matrix", "distort_coeffs", "R_gimbal2imubody", "R_camera2gimbal", "t_camera2gimbal"};
const std::set<std::string> kTrackerKeys = {
  "enemy_color", "min_detect_count", "max_temp_lost_count", "outpost_max_temp_lost_count"};
const std::set<std::string> kAimKeys = {
  "yaw_offset", "left_yaw_offset", "right_yaw_offset", "pitch_offset", "comming_angle",
  "leaving_angle", "decision_speed", "high_speed_delay_time", "low_speed_delay_time", "nail_mode",
  "nail_spin_on", "nail_spin_off", "nail_fire_window_deg", "first_tolerance", "second_tolerance",
  "judge_distance", "auto_fire", "fire_gap_time", "predict_time"};
const std::set<std::string> kLobKeys = {
  "target_x", "target_y", "target_z", "base_x", "base_y", "base_z", "base_yaw", "muzzle_dx",
  "muzzle_dy", "muzzle_dz", "friction", "friction_coeff", "pitch_max_deg", "yaw_tol", "pitch_tol",
  "aim_timeout_s", "pose_timeout_s", "pitch_sign", "yaw_sign", "open_loop", "module_unlocked",
  "module_docked", "module_assembling", "windmill_x", "windmill_y", "windmill_radius",
  "windmill_z_min", "windmill_z_max"};
const std::set<std::string> kBoardKeys = {"mode", "bullet_speed", "deploy"};
const std::set<std::string> kCameraKeys = {"exposure_ms", "gain"};

// Foxglove 参数面板里列出的 key（都属于上面的热加载组）
const std::vector<std::string> kPanelKeys = {
  "mode", "enemy_color", "exposure_ms", "gain", "bullet_speed",
  "yaw_offset", "pitch_offset", "comming_angle", "leaving_angle", "decision_speed",
  "high_speed_delay_time", "low_speed_delay_time",
  "nail_mode", "nail_spin_on", "nail_spin_off", "nail_fire_window_deg",
  "first_tolerance", "second_tolerance", "judge_distance", "auto_fire",
  "min_detect_count", "max_temp_lost_count", "outpost_max_temp_lost_count",
  "fire_gap_time", "predict_time",
  "R_camera2gimbal", "t_camera2gimbal",
  "target_x", "target_y", "target_z", "base_x", "base_y", "base_yaw", "pitch_max_deg",
  "yaw_tol", "pitch_tol", "open_loop", "module_unlocked", "module_docked", "module_assembling",
  "debug.image_hz", "debug.image_scale", "debug.jpeg_quality", "debug.raw"};

const cv::Scalar kYellow{0, 255, 255}, kGreen{0, 255, 0}, kRed{0, 0, 255}, kBlue{255, 0, 0},
  kGray{160, 160, 160};

bool any_of(const std::set<std::string> & changed, const std::set<std::string> & group)
{
  for (const auto & k : changed)
    if (group.count(k)) return true;
  return false;
}

double ms(Clock::duration d) { return std::chrono::duration<double, std::milli>(d).count(); }

std::optional<io::Mode> parse_mode(const std::string & s)
{
  for (size_t i = 0; i < io::MODES.size(); ++i)
    if (io::MODES[i] == s) return static_cast<io::Mode>(i);
  return std::nullopt;
}

// 0xA5 的 z：0 不控 / 1 跟踪 / 2 开火（与旧 /gimbal_command.z 同义）
int to_z(const io::Command & c) { return c.shoot ? 2 : (c.control ? 1 : 0); }

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

// 光心（camera_matrix 的 cx, cy），vis 上画十字，对应旧 final_img 的蓝圈
std::optional<cv::Point2f> principal_point(const YAML::Node & y)
{
  if (!y["camera_matrix"] || y["camera_matrix"].size() < 6) return std::nullopt;
  return cv::Point2f(y["camera_matrix"][2].as<float>(), y["camera_matrix"][5].as<float>());
}

// 回放时 IMU 来自 txt：采集线程按时间戳存，主线程按同一时间戳取
class ReplayImu
{
public:
  void put(Clock::time_point t, const Eigen::Quaterniond & q)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    q_[t] = q;
  }
  Eigen::Quaterniond take(Clock::time_point t)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = q_.find(t);
    Eigen::Quaterniond q = it == q_.end() ? Eigen::Quaterniond::Identity() : it->second;
    q_.erase(q_.begin(), q_.upper_bound(t));
    return q;
  }

private:
  std::mutex mutex_;
  std::map<Clock::time_point, Eigen::Quaterniond> q_;
};

void put_ekf_debug(nlohmann::json & data, const auto_aim::Target & target)
{
  const auto & dbg = target.ekf().data;
  for (const char * k :
       {"residual_yaw", "residual_pitch", "residual_distance", "residual_angle", "nis", "nees",
        "nis_fail", "nees_fail", "recent_nis_failures"}) {
    auto it = dbg.find(k);
    if (it != dbg.end()) data[k] = it->second;
  }
}

// 字段对齐 mt_auto_aim_debug + 本队英雄字段；Foxglove /debug 与 PlotJuggler 共用
nlohmann::json plot_auto_aim(
  const Eigen::Vector3d & ypr, const std::list<auto_aim::Armor> & armors,
  const std::list<auto_aim::Target> & targets, const auto_aim::Solver & solver,
  const auto_aim::Tracker & tracker, const auto_aim::Aimer & aimer, const io::Command & command,
  const io::HeroBoard & board)
{
  nlohmann::json data;
  data["tracker_state"] = tracker.state();
  data["armor_num"] = static_cast<int>(armors.size());
  data["target_num"] = static_cast<int>(targets.size());

  data["gimbal_yaw"] = ypr[0] * 57.3;
  data["gimbal_pitch"] = ypr[1] * 57.3;
  data["gimbal_roll"] = ypr[2] * 57.3;
  data["bullet_speed"] = board.bullet_speed;

  data["cmd_yaw"] = command.yaw * 57.3;
  data["cmd_pitch"] = command.pitch * 57.3;          // 同济世界系：抬头为负
  data["cmd_pitch_hero"] = (-command.pitch) * 57.3;  // HeroBoard 翻号后下发
  data["cmd_control"] = command.control;
  data["cmd_shoot"] = command.shoot;
  data["cmd_z"] = to_z(command);

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

  // 原始观测：取最左装甲（同济习惯），与 EKF 的 x/y/a 对照看 PnP 跳变
  if (!armors.empty()) {
    auto armor = armors.front();
    for (const auto & a : armors)
      if (a.center.x < armor.center.x) armor = a;
    solver.solve(armor);
    data["armor_x"] = armor.xyz_in_world[0];
    data["armor_y"] = armor.xyz_in_world[1];
    data["armor_z"] = armor.xyz_in_world[2];
    data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    data["armor_cx"] = armor.center.x;
    data["armor_cy"] = armor.center.y;
  }

  // EKF 状态 x vx y vy z vz a w r l h（w 在下标 7）
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

// vis：黄=敌方色 YOLO 检出（灰=非敌方，被 Tracker 滤掉），绿=EKF 四块板重投影，红=Aimer 瞄准板（蓝=无效）
tools::Overlay overlay_auto_aim(
  const std::list<auto_aim::Armor> & detected, auto_aim::Color enemy,
  const std::list<auto_aim::Target> & targets, const auto_aim::Solver & solver,
  const auto_aim::Aimer & aimer)
{
  tools::Overlay ov;
  for (const auto & a : detected) {
    const auto color = a.color == enemy ? kYellow : kGray;
    ov.poly(a.points, color);
    ov.text(
      fmt::format(
        "{} {} {:.2f}", auto_aim::COLORS[a.color], auto_aim::ARMOR_NAMES[a.name], a.confidence),
      a.points.empty() ? a.center : a.points.front() - cv::Point2f(0, 8), color);
  }
  if (targets.empty()) return ov;
  const auto & target = targets.front();
  for (const Eigen::Vector4d & xyza : target.armor_xyza_list()) {
    ov.poly(solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name), kGreen);
  }
  const auto & aim = aimer.debug_aim_point;
  ov.poly(
    solver.reproject_armor(aim.xyza.head(3), aim.xyza[3], target.armor_type, target.name),
    aim.valid ? kRed : kBlue);
  return ov;
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
  tools::LiveConfig live(config_path);
  const auto & cfg = live.path();  // 各模块都读合并后的 yaml，热加载才能生效
  const auto yaml0 = live.node();

  const auto video = cli.get<std::string>("video");
  const bool replay_mode = !video.empty();
  const bool fake_board = cli.get<bool>("fake-board") || replay_mode;

  // —— 调试输出 ——
  auto dbg_opt = tools::DebugOptions::from_yaml(yaml0["debug"]);
  if (cli.has("imshow")) dbg_opt.imshow = cli.get<bool>("imshow");
  if (cli.has("plot")) dbg_opt.plot = cli.get<bool>("plot");
  if (cli.has("plot-host")) dbg_opt.plot_host = cli.get<std::string>("plot-host");
  if (cli.has("plot-port")) dbg_opt.plot_port = cli.get<int>("plot-port");
  tools::DebugSink sink(dbg_opt, &live, kPanelKeys);

  // —— 硬件 / 数据源 ——
  std::unique_ptr<io::Camera> camera;
  std::unique_ptr<tools::Replay> replay;
  ReplayImu replay_imu;
  if (replay_mode) {
    replay = std::make_unique<tools::Replay>(video, cli.get<bool>("video-fast"));
  } else {
    camera = std::make_unique<io::Camera>(cfg);
  }
  io::HeroBoard board(cfg, !fake_board);
  if (cli.has("mode")) {
    if (auto m = parse_mode(cli.get<std::string>("mode"))) {
      board.set_yaml_mode(*m);
    } else {
      tools::logger()->warn("--mode={} 不认识，沿用 yaml", cli.get<std::string>("mode"));
    }
  }

  std::unique_ptr<tools::Recorder> recorder;
  if (cli.get<bool>("record") && !replay_mode) {
    double fps = 60;
    if (yaml0["debug"] && yaml0["debug"]["record_fps"]) fps = yaml0["debug"]["record_fps"].as<double>();
    recorder = std::make_unique<tools::Recorder>(fps);
    tools::logger()->info("[Record] 录制到 records/（上限 {:.0f}fps）", fps);
  }

  // —— 算法对象：热加载时按组重建 ——
  if (yaml0["yolo_name"].as<std::string>() != "yolov5") {
    tools::logger()->warn("MultiThreadDetector 按 yolov5 输出解析，当前 yolo_name 可能解析错");
  }
  // 在途 3 帧：GPU 吞吐够用，延迟上限≈3/推理fps，比同济默认 16 低一个量级
  auto_aim::multithread::MultiThreadDetector detector(cfg, false, 3);
  tools::logger()->info("[OV] 自瞄推理设备 {}", detector.device());

  auto solver = std::make_unique<auto_aim::Solver>(cfg);
  auto tracker = std::make_unique<auto_aim::Tracker>(cfg, *solver);
  auto aimer = std::make_unique<auto_aim::Aimer>(cfg);
  auto shooter = std::make_unique<auto_aim::Shooter>(cfg);

  auto_buff::Buff_Detector buff_detector(cfg);
  auto buff_solver = std::make_unique<auto_buff::Solver>(cfg);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto buff_aimer = std::make_unique<auto_buff::Aimer>(cfg);

  auto lob = std::make_unique<lob_shot::Manager>(load_lob(yaml0));
  bool lob_armed = false;
  auto center = principal_point(yaml0);
  auto enemy_of = [](const YAML::Node & y) {
    return y["enemy_color"].as<std::string>() == "red" ? auto_aim::Color::red : auto_aim::Color::blue;
  };
  auto enemy = enemy_of(yaml0);

  auto reload = [&](const std::set<std::string> & changed) {
    const auto y = live.node();
    std::vector<std::string> done, restart;
    if (any_of(changed, kSolverKeys)) {
      tracker.reset();  // Tracker 引用着旧 Solver，先拆
      solver = std::make_unique<auto_aim::Solver>(cfg);
      buff_solver = std::make_unique<auto_buff::Solver>(cfg);
      tracker = std::make_unique<auto_aim::Tracker>(cfg, *solver);
      center = principal_point(y);
      done.push_back("solver+tracker");
    } else if (any_of(changed, kTrackerKeys)) {
      tracker = std::make_unique<auto_aim::Tracker>(cfg, *solver);
      done.push_back("tracker");
    }
    enemy = enemy_of(y);
    if (any_of(changed, kAimKeys)) {
      aimer = std::make_unique<auto_aim::Aimer>(cfg);
      shooter = std::make_unique<auto_aim::Shooter>(cfg);
      buff_aimer = std::make_unique<auto_buff::Aimer>(cfg);
      done.push_back("aimer/shooter/buff_aimer");
    }
    if (any_of(changed, kLobKeys)) {
      lob = std::make_unique<lob_shot::Manager>(load_lob(y));
      lob_armed = false;
      done.push_back("lob");
    }
    if (any_of(changed, kBoardKeys)) {
      board.apply_yaml(cfg);
      done.push_back("board");
    }
    if (any_of(changed, kCameraKeys) && camera) {
      camera->set_exposure_gain(y["exposure_ms"].as<double>(), y["gain"].as<double>());
      done.push_back("camera");
    }
    if (changed.count("debug")) {
      sink.reconfigure(y["debug"]);
      done.push_back("debug");
    }
    for (const auto & k : changed) {
      const bool hot = kSolverKeys.count(k) || kTrackerKeys.count(k) || kAimKeys.count(k) ||
                       kLobKeys.count(k) || kBoardKeys.count(k) || kCameraKeys.count(k) ||
                       k == "debug";
      if (!hot) restart.push_back(k);
    }
    tools::logger()->info(
      "[热加载] 已生效 [{}]{}", fmt::join(done, ", "),
      restart.empty() ? "" : fmt::format("；需重启才生效 [{}]", fmt::join(restart, ", ")));
    sink.publish_params();
  };

  // —— 采集线程 ——
  struct Frame
  {
    cv::Mat img;
    Clock::time_point t;
  };
  tools::ThreadSafeQueue<Frame, true> latest(1);  // 非自瞄模式只要最新一帧
  std::atomic<bool> feed_detector{false};
  std::atomic<bool> replay_done{false};
  std::atomic<int> cam_frames{0};

  std::thread grab([&] {
    cv::Mat img;
    Clock::time_point t;
    Eigen::Quaterniond q;
    while (!exiter.exit()) {
      if (replay) {
        if (!replay->read(img, t, q)) {
          replay_done = true;
          tools::logger()->info("[Replay] 放完");
          return;
        }
        img = img.clone();  // VideoCapture 会复用缓冲，异步推理必须拷走
        replay_imu.put(t, q);
      } else {
        camera->read(img, t);  // 无帧 100ms 超时返回空，保证 Ctrl+C 能退
        if (img.empty()) continue;
      }
      ++cam_frames;
      if (feed_detector) {
        detector.push(img, t);
      } else {
        latest.push({img, t});
      }
    }
  });

  auto imu_at = [&](Clock::time_point t) {
    return replay ? replay_imu.take(t) : board.imu_at(t - 1ms);
  };

  // —— 主循环 ——
  auto last_mode = io::Mode::idle;
  bool first = true;
  int last_cam = -1;
  const auto t0 = Clock::now();
  auto prev = t0;
  auto last_poll = t0;
  int frame = 0;
  double fps = 0, cam_fps = 0;
  auto fps_t0 = t0;
  int fps_n = 0, cam_n0 = 0;

  auto tick_fps = [&] {
    ++frame;
    ++fps_n;
    const auto now = Clock::now();
    const double dt = std::chrono::duration<double>(now - fps_t0).count();
    if (dt >= 0.5) {
      fps = fps_n / dt;
      const int n = cam_frames.load();
      cam_fps = (n - cam_n0) / dt;
      cam_n0 = n;
      fps_n = 0;
      fps_t0 = now;
    }
  };

  // 每条曲线都带的公共字段
  auto base = [&](nlohmann::json & d, io::Mode mode) {
    d["t"] = tools::delta_time(Clock::now(), t0);
    d["frame"] = frame;
    d["fps"] = fps;
    d["cam_fps"] = cam_fps;
    d["mode"] = io::MODES[mode];
    d["cam_id"] = board.camera_id();
    double jy = 0, jp = 0;
    board.latest_joint(jy, jp);
    d["joint_yaw"] = jy * 57.3;
    d["joint_pitch"] = jp * 57.3;  // 英雄 URDF，抬头为正
  };

  auto header = [&](io::Mode mode, double latency_ms, const std::string & state) {
    std::vector<std::string> h;
    h.push_back(
      fmt::format("#{} {:.0f}fps cam {:.0f} Latency {:.1f}ms", frame, fps, cam_fps, latency_ms));
    h.push_back(fmt::format("{} {}{}", io::MODES[mode], state, replay ? "  REPLAY" : ""));
    return h;
  };

  while (!exiter.exit()) {
    if (Clock::now() - last_poll > 500ms) {
      last_poll = Clock::now();
      if (auto changed = live.poll(); !changed.empty()) reload(changed);
    }

    const io::Mode mode = board.mode;
    const bool aim_mode = mode == io::Mode::auto_aim || mode == io::Mode::outpost;
    if (first || last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      // 离开自瞄时把在途推理收掉，回来时不会吃到几秒前的旧帧
      if (!aim_mode) {
        feed_detector = false;
        while (detector.debug_pop_for(0ms)) {
        }
      }
      if (mode != io::Mode::lob) lob_armed = false;
      last_mode = mode;
      first = false;
    }
    feed_detector = aim_mode;

    if (board.camera_id() != last_cam) {
      last_cam = board.camera_id();
      if (last_cam == 1) {
        tools::logger()->warn("电控切长焦，本轮仅短焦：仍用短焦相机与内参");
      } else {
        tools::logger()->info("英雄相机短焦（上置）");
      }
    }

    // 吊射：不依赖图像，只跑状态机；pitch 已是抬头为正，勿走同济翻号
    if (mode == io::Mode::lob) {
      if (!lob_armed) {
        lob->request_trigger();
        lob_armed = true;
      }
      double jy = 0, jp = 0;
      board.latest_joint(jy, jp);
      lob->set_feedback(jy, lob_shot::joint_pitch_to_nose_up(jp));
      if (board.bullet_speed > 1.0) lob->set_bullet_speed(board.bullet_speed);
      const auto now = Clock::now();
      const double dt = std::chrono::duration<double>(now - prev).count();
      prev = now;
      auto lob_cmd = lob->tick(dt > 0 ? dt : 0.02);
      board.send({lob_cmd.control, lob_cmd.shoot, lob_cmd.yaw, lob_cmd.pitch}, false);
      tick_fps();

      nlohmann::json d;
      base(d, mode);
      d["status"] = lob->status();
      d["yaw_deg"] = lob->last_yaw_deg();
      d["pitch_deg"] = lob->last_pitch_deg();
      d["high_unused_deg"] = lob->last_high_deg();
      d["z"] = lob_shot::to_z(lob_cmd);
      d["deploy"] = lob_shot::to_deploy(lob_cmd);
      d["fire_latched"] = lob->fire_latched();
      d["cmd_yaw"] = lob_cmd.yaw * 57.3;
      d["cmd_pitch"] = lob_cmd.pitch * 57.3;  // 英雄抬头为正
      d["cmd_control"] = lob_cmd.control;
      d["cmd_shoot"] = lob_cmd.shoot;
      d["bullet_speed"] = board.bullet_speed;
      sink.plot(d);

      Frame f;  // 吊射不用图，但照样推原图方便看场地
      if (sink.want_image() && latest.pop_for(f, 0ms)) {
        tools::Overlay ov;
        ov.header = header(mode, 0, lob->status());
        ov.header.push_back(fmt::format(
          "lob yaw {:+.2f} pitch {:+.2f} z{}", lob->last_yaw_deg(), lob->last_pitch_deg(),
          lob_shot::to_z(lob_cmd)));
        sink.image(f.img, std::move(ov));
      }
      std::this_thread::sleep_for(20ms);
      continue;
    }
    prev = Clock::now();

    // —— 自瞄 / 前哨 ——
    if (aim_mode) {
      auto got = detector.debug_pop_for(100ms);
      if (!got) {
        if (replay_done) break;
        continue;
      }
      auto [img, armors, t] = std::move(*got);
      const auto t_det = Clock::now();
      tick_fps();

      Eigen::Quaterniond q = imu_at(t);
      if (recorder) recorder->record(img, q, t);
      if (board.take_reset()) tracker = std::make_unique<auto_aim::Tracker>(cfg, *solver);
      solver->set_R_gimbal2world(q);
      Eigen::Vector3d ypr = tools::eulers(solver->R_gimbal2world(), 2, 1, 0);
      const auto detected = armors;  // track() 会就地删掉非敌方颜色；调试要看 YOLO 全部检出
      auto targets = tracker->track(armors, t);
      auto command = aimer->aim(targets, t, board.bullet_speed);
      command.shoot = shooter->shoot(command, *aimer, targets, ypr);
      board.send(command);  // 同济抬头为负 → 0xA5 翻号
      const auto t_sent = Clock::now();
      const double latency = ms(t_sent - t);

      auto d = plot_auto_aim(ypr, armors, targets, *solver, *tracker, *aimer, command, board);
      base(d, mode);
      d["detect_num"] = static_cast<int>(detected.size());
      d["latency_ms"] = latency;  // 曝光到下发：旧 final_img 的 Latency
      d["t_aim_ms"] = ms(t_sent - t_det);
      d["det_drop"] = detector.dropped();
      sink.plot(d);

      if (sink.want_image()) {
        auto ov = overlay_auto_aim(detected, enemy, targets, *solver, *aimer);
        ov.center = center;
        ov.header = header(mode, latency, fmt::format("[{}]", tracker->state()));
        ov.header.push_back(fmt::format(
          "cmd yaw {:+.2f} pitch {:+.2f} z{}{}", command.yaw * 57.3, -command.pitch * 57.3,
          to_z(command), aimer->nail_latched() ? "  NAIL" : ""));
        sink.image(img, std::move(ov));
      }
      continue;
    }

    // —— 打符 / idle：同步处理最新一帧 ——
    Frame f;
    if (!latest.pop_for(f, 100ms)) {
      if (replay_done) break;
      continue;
    }
    tick_fps();
    Eigen::Quaterniond q = imu_at(f.t);
    if (recorder) recorder->record(f.img, q, f.t);

    if (mode == io::Mode::small_buff || mode == io::Mode::big_buff) {
      buff_solver->set_R_gimbal2world(q);
      auto power_runes = buff_detector.detect(f.img);
      buff_solver->solve(power_runes);
      io::Command buff_command;
      bool unsolve = true;
      if (mode == io::Mode::small_buff) {
        buff_small_target.get_target(power_runes, f.t);
        auto target_copy = buff_small_target;
        unsolve = target_copy.is_unsolve();
        buff_command = buff_aimer->aim(target_copy, f.t, board.bullet_speed, true);
      } else {
        buff_big_target.get_target(power_runes, f.t);
        auto target_copy = buff_big_target;
        unsolve = target_copy.is_unsolve();
        buff_command = buff_aimer->aim(target_copy, f.t, board.bullet_speed, true);
      }
      board.send(buff_command);  // 同济抬头为负 → 0xA5 翻号
      const double latency = ms(Clock::now() - f.t);

      nlohmann::json d;
      base(d, mode);
      d["latency_ms"] = latency;
      d["buff_unsolve"] = unsolve;
      d["rune_ok"] = power_runes.has_value();
      d["cmd_yaw"] = buff_command.yaw * 57.3;
      d["cmd_pitch"] = buff_command.pitch * 57.3;
      d["cmd_pitch_hero"] = (-buff_command.pitch) * 57.3;
      d["cmd_control"] = buff_command.control;
      d["cmd_shoot"] = buff_command.shoot;
      d["cmd_z"] = to_z(buff_command);
      d["bullet_speed"] = board.bullet_speed;
      sink.plot(d);

      if (sink.want_image()) {
        tools::Overlay ov;
        ov.center = center;
        if (power_runes) {
          // 绿=当前要打的扇叶，灰=其余扇叶，黄=R 标
          for (size_t i = 0; i < power_runes->fanblades.size(); ++i) {
            const auto & pts = power_runes->fanblades[i].points;
            if (!pts.empty()) ov.poly(pts, i == 0 ? kGreen : kGray);
          }
          ov.text("R", power_runes->r_center, kYellow);
        }
        ov.header = header(mode, latency, unsolve ? "unsolve" : "solved");
        ov.header.push_back(fmt::format(
          "cmd yaw {:+.2f} pitch {:+.2f} z{}", buff_command.yaw * 57.3,
          -buff_command.pitch * 57.3, to_z(buff_command)));
        sink.image(f.img, std::move(ov));
      }
      continue;
    }

    // idle：不下发，只看图和姿态（不接电控也能确认相机/曝光）
    nlohmann::json d;
    base(d, mode);
    Eigen::Vector3d ypr = tools::eulers(q.toRotationMatrix(), 2, 1, 0);
    d["gimbal_yaw"] = ypr[0] * 57.3;
    d["gimbal_pitch"] = ypr[1] * 57.3;
    sink.plot(d);
    if (sink.want_image()) {
      tools::Overlay ov;
      ov.center = center;
      ov.header = header(mode, ms(Clock::now() - f.t), "");
      sink.image(f.img, std::move(ov));
    }
  }

  feed_detector = false;
  grab.join();
  return 0;
}
