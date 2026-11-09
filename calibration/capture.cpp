// 标定采图：相机 + 英雄串口姿态，保存 N.jpg（+ 手眼用的 N.txt）。不依赖 ROS，不发送任何控制帧。
//
// 与旧 ROS capture_node 相比：
// - 直接读相机和串口，不需要先跑整套自瞄；姿态按图像时间戳插值（与 build/hero 相同的 hero::Serial），
//   不是“最近一次关节角”；
// - 预览检测在后台线程只处理最新一帧且降采样，界面与相机读取不被检测阻塞；保存时才做全分辨率复检；
// - 云台未静止、姿态超时、板未完整检出时拒绝保存；
// - 不清空输出目录；--append 续写编号。
#include <fmt/core.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <opencv2/opencv.hpp>
#include <thread>

#include "calibration/calib_common.hpp"
#include "io/camera.hpp"
#include "io/hero/hero_serial.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

namespace fs = std::filesystem;

namespace
{
const std::string keys =
  "{help h usage ? |                                         | 输出命令行参数说明 }"
  "{@output        |                                         | 输出目录（必填）：N.jpg，手眼另有 N.txt }"
  "{board b        | configs/calibration_chessboard_11x8.yaml | 标定板配置：大棋盘格或 configs/calibration.yaml 小圆点板 }"
  "{config c       | configs/hero.yaml                       | 相机与串口配置，默认与自瞄相同 }"
  "{exposure-ms    | -1                                      | 临时覆盖曝光（ms），<0 用 config }"
  "{gain           | -1                                      | 临时覆盖增益，<0 用 config }"
  "{no-serial      | false                                   | 不接电控：只拍内参照片，不写 N.txt }"
  "{auto           | false                                   | 自动保存（满足条件且与已存样本差异够大时） }"
  "{min-angle      | 2.0                                     | 手眼自动保存：与已存样本的最小 yaw/pitch 差（度） }"
  "{still-deg      | 0.05                                    | 判定静止：图像前 50 ms 内云台转角上限（度） }"
  "{count          | 0                                       | 存够 N 张后退出，0=不限 }"
  "{append         | false                                   | 输出目录已有图片时续写编号 }"
  "{preview p      | auto                                    | auto=窗口（s 保存，a 切换自动，q 退出）；none=无窗口，只自动保存 }";

volatile std::sig_atomic_t interrupted = 0;

fs::path resolve(const std::string & path)
{
  fs::path p(path);
  if (p.is_relative() && !fs::exists(p)) p = fs::path(CALIB_SOURCE_ROOT) / p;
  return p;
}

int next_index(const fs::path & folder)
{
  int index = 0;
  while (fs::exists(folder / fmt::format("{}.jpg", index + 1))) index++;
  return index + 1;
}

struct Frame
{
  cv::Mat image;
  std::chrono::steady_clock::time_point stamp;
  uint64_t id = 0;
};

// 采集时刻的云台姿态：hero 约定（yaw 绕 +Z，pitch 抬头为正），与运行时 Solver 输入相同
struct Pose
{
  Eigen::Quaterniond q;
  double yaw = 0, pitch = 0;  // rad
  double motion_deg = 0;      // 图像前 50 ms 内的转角
};

struct Analysis
{
  Frame frame;
  bool ok = false;
  std::vector<cv::Point2f> points;
  std::optional<Pose> pose;
};

// 样本在“姿态空间”（手眼）或“画面空间”（内参）的位置，用于自动保存时挑选差异大的样本
struct Signature
{
  double a = 0, b = 0, c = 0;
};
}  // namespace

int main(int argc, char ** argv)
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help") || cli.get<std::string>(0).empty()) {
    cli.printMessage();
    return cli.has("help") ? 0 : 2;
  }
  const fs::path output = cli.get<std::string>(0);
  const auto board_path = resolve(cli.get<std::string>("board"));
  const auto config_path = resolve(cli.get<std::string>("config"));
  const bool use_serial = !cli.get<bool>("no-serial");
  const bool headless = cli.get<std::string>("preview") == "none";
  std::atomic<bool> automatic{cli.get<bool>("auto") || headless};
  const double min_angle = cli.get<double>("min-angle") * M_PI / 180;
  const double still = cli.get<double>("still-deg") * M_PI / 180;
  const int count_limit = cli.get<int>("count");
  if (!cli.check()) {
    cli.printErrors();
    return 2;
  }

  try {
    const auto board = calib::load_board_config(board_path.string());
    auto config = YAML::LoadFile(config_path.string());
    if (cli.get<double>("exposure-ms") > 0) config["exposure_ms"] = cli.get<double>("exposure-ms");
    if (cli.get<double>("gain") >= 0) config["gain"] = cli.get<double>("gain");

    fs::create_directories(output);
    int index = next_index(output);
    if (index > 1 && !cli.get<bool>("append"))
      throw std::runtime_error(
        fmt::format("{} 已有 {} 张图片；换一个新目录，或加 --append 续写", output.string(), index - 1));

    // 求解用的板配置：标定板参数 + 采集时实际使用的相机内参，求解时 --config-path 指向它即可
    {
      auto snapshot = YAML::LoadFile(board_path.string());
      snapshot["camera_matrix"] = config["camera_matrix"];
      snapshot["distort_coeffs"] = config["distort_coeffs"];
      snapshot["R_gimbal2imubody"] = config["R_gimbal2imubody"];
      std::ofstream out(output / "board.yaml");
      out << "# capture 生成：" << board_path.string() << " + " << config_path.string() << " 的相机内参\n"
          << YAML::Dump(snapshot) << '\n';
    }
    char camera_yaml[] = "/tmp/hero-capture-XXXXXX";
    const int fd = mkstemp(camera_yaml);
    if (fd < 0) throw std::runtime_error("Cannot create camera configuration");
    close(fd);
    std::ofstream(camera_yaml) << YAML::Dump(config);

    std::signal(SIGINT, [](int) { interrupted = 1; });
    std::signal(SIGTERM, [](int) { interrupted = 1; });
    std::unique_ptr<hero::Serial> serial;
    if (use_serial) serial = std::make_unique<hero::Serial>(config["serial"]);
    io::Camera camera(camera_yaml);
    fs::remove(camera_yaml);
    fmt::print(
      "标定板 {}；输出 {}（从 {}.jpg 起）；{}；{}\n", board.describe(), output.string(), index,
      use_serial ? "串口姿态 → 手眼数据（N.jpg + N.txt）" : "无串口 → 内参照片（只有 N.jpg）",
      headless ? "无窗口，自动保存" : "窗口：s 保存，a 切换自动，q 退出");

    const auto pose_at = [&](std::chrono::steady_clock::time_point stamp) -> std::optional<Pose> {
      if (!serial) return std::nullopt;
      const auto q = serial->imu_at(stamp);
      const auto before = serial->imu_at(stamp - std::chrono::milliseconds(50));
      if (!q || !before) return std::nullopt;
      const Eigen::Vector3d ypr = tools::eulers(q->toRotationMatrix(), 2, 1, 0);
      return Pose{*q, ypr[0], -ypr[1], q->angularDistance(*before)};
    };

    // ---- 预览检测线程：只处理最新一帧 ----
    std::mutex frame_mutex;
    std::condition_variable frame_ready;
    std::optional<Frame> latest_frame;
    std::mutex analysis_mutex;
    Analysis latest;
    std::atomic<int> analysed{0};
    std::atomic<bool> stop{false};

    // ---- 保存线程 ----
    std::mutex save_mutex;
    std::condition_variable save_ready;
    std::deque<Analysis> save_queue;
    std::atomic<bool> saving{false};
    std::vector<Signature> saved;
    std::mutex saved_mutex;
    std::atomic<int> saved_count{0};

    const auto signature = [&](const Analysis & a) {
      if (a.pose) return Signature{a.pose->yaw, a.pose->pitch, 0};
      const auto box = cv::boundingRect(a.points);
      const double w = a.frame.image.cols, h = a.frame.image.rows;
      return Signature{(box.x + box.width / 2.0) / w, (box.y + box.height / 2.0) / h, box.area() / (w * h)};
    };
    const auto novel = [&](const Signature & s) {
      std::lock_guard<std::mutex> lock(saved_mutex);
      for (const auto & o : saved) {
        const double d = use_serial ? std::max(std::abs(s.a - o.a), std::abs(s.b - o.b))
                                    : std::max({std::abs(s.a - o.a), std::abs(s.b - o.b), 3 * std::abs(s.c - o.c)});
        if (d < (use_serial ? min_angle : 0.08)) return false;
      }
      return true;
    };
    // 返回不能保存的原因；空字符串表示可以保存
    const auto reject_reason = [&](const Analysis & a) -> std::string {
      if (!a.ok) return "未完整检出标定板";
      if (!use_serial) return "";
      if (!a.pose) return "串口姿态超时或缺失（电控是否在发 0x5A 反馈？）";
      if (a.pose->motion_deg > still)
        return fmt::format("云台未静止（50 ms 内转了 {:.3f}°）", a.pose->motion_deg * 180 / M_PI);
      return "";
    };
    const auto request_save = [&](Analysis a) {
      std::lock_guard<std::mutex> lock(save_mutex);
      if (saving || !save_queue.empty()) return;
      saving = true;
      save_queue.push_back(std::move(a));
      save_ready.notify_one();
    };

    std::thread detector_thread([&] {
      calib::BoardDetector detector(board);
      uint64_t last_id = 0;
      while (!stop) {
        Frame frame;
        {
          std::unique_lock<std::mutex> lock(frame_mutex);
          frame_ready.wait_for(lock, std::chrono::milliseconds(100), [&] {
            return stop || (latest_frame && latest_frame->id != last_id);
          });
          if (stop) break;
          if (!latest_frame || latest_frame->id == last_id) continue;
          frame = *latest_frame;
          last_id = frame.id;
        }
        Analysis a;
        a.frame = frame;
        a.ok = detector.detect(frame.image, a.points, false);
        a.pose = pose_at(frame.stamp);
        ++analysed;
        if (automatic && reject_reason(a).empty() && novel(signature(a))) request_save(a);
        std::lock_guard<std::mutex> lock(analysis_mutex);
        latest = std::move(a);
      }
    });

    std::thread saver_thread([&] {
     try {
      calib::BoardDetector detector(board);
      const std::vector<int> jpeg = {cv::IMWRITE_JPEG_QUALITY, board.jpeg_quality};
      while (true) {
        Analysis a;
        {
          std::unique_lock<std::mutex> lock(save_mutex);
          save_ready.wait(lock, [&] { return stop || !save_queue.empty(); });
          if (save_queue.empty()) break;
          a = std::move(save_queue.front());
          save_queue.pop_front();
        }
        // 全分辨率复检：与离线求解器使用同一检测结果
        a.ok = detector.detect(a.frame.image, a.points, true);
        if (const auto reason = reject_reason(a); !reason.empty()) {
          fmt::print("[未保存] {}\n", reason);
        } else {
          const auto stem = output / std::to_string(index);
          if (!cv::imwrite(stem.string() + ".jpg", a.frame.image, jpeg))
            throw std::runtime_error("Cannot write " + stem.string() + ".jpg");
          if (a.pose) {
            std::ofstream q(stem.string() + ".txt");
            q << fmt::format("{} {} {} {}\n", a.pose->q.w(), a.pose->q.x(), a.pose->q.y(), a.pose->q.z());
            if (!q) throw std::runtime_error("Cannot write " + stem.string() + ".txt");
            fmt::print(
              "[{}] 已保存  yaw {:.2f}°  pitch {:.2f}°（抬头为正）\n", index, a.pose->yaw * 180 / M_PI,
              a.pose->pitch * 180 / M_PI);
          } else {
            fmt::print("[{}] 已保存\n", index);
          }
          {
            std::lock_guard<std::mutex> lock(saved_mutex);
            saved.push_back(signature(a));
          }
          index++;
          saved_count++;
        }
        saving = false;
      }
     } catch (const std::exception & error) {
      fmt::print(stderr, "[capture] 保存失败，停止采集：{}\n", error.what());
      interrupted = 1;
     }
    });

    uint64_t frame_id = 0, frames_read = 0;
    auto last_draw = std::chrono::steady_clock::now(), last_report = last_draw;
    int last_analysed = 0;
    uint64_t last_frames = 0;
    const std::string window = "capture: s=save a=auto q=quit";
    try {
      while (!interrupted && (count_limit == 0 || saved_count < count_limit)) {
        Frame frame;
        camera.read(frame.image, frame.stamp);
        if (frame.image.empty()) continue;
        frame.id = ++frame_id;
        ++frames_read;
        {
          std::lock_guard<std::mutex> lock(frame_mutex);
          latest_frame = frame;  // cv::Mat 共享数据，不拷贝像素
        }
        frame_ready.notify_one();

        const auto now = std::chrono::steady_clock::now();
        if (now - last_report >= std::chrono::seconds(1)) {
          const double dt = std::chrono::duration<double>(now - last_report).count();
          Analysis a;
          {
            std::lock_guard<std::mutex> lock(analysis_mutex);
            a = latest;
          }
          // 三项条件分开显示，便于判断是板、电控还是云台的问题
          const auto attitude = !use_serial ? std::string("不使用")
                                : a.pose    ? fmt::format(
                                             "yaw {:.2f}° pitch {:.2f}° 50ms转角 {:.3f}°{}", a.pose->yaw * 180 / M_PI,
                                             a.pose->pitch * 180 / M_PI, a.pose->motion_deg * 180 / M_PI,
                                             a.pose->motion_deg > still ? "（未静止）" : "")
                                            : std::string("超时（电控未发 0x5A 反馈？）");
          fmt::print(
            "相机 {:.0f} FPS  检测 {:.0f} FPS  板 {}  姿态 {}  已存 {}  自动 {}  → {}\n",
            (frames_read - last_frames) / dt, (analysed - last_analysed) / dt, a.ok ? "检出" : "未检出", attitude,
            saved_count.load(), automatic ? "开" : "关", reject_reason(a).empty() ? "可保存" : "不可保存");
          last_frames = frames_read;
          last_analysed = analysed;
          last_report = now;
        }
        if (headless || now - last_draw < std::chrono::milliseconds(33)) continue;
        last_draw = now;

        Analysis a;
        {
          std::lock_guard<std::mutex> lock(analysis_mutex);
          a = latest;
        }
        cv::Mat preview;
        cv::resize(frame.image, preview, {}, 0.5, 0.5, cv::INTER_AREA);
        auto points = a.points;
        for (auto & p : points) p *= 0.5f;
        cv::drawChessboardCorners(preview, board.pattern_size(), points, a.ok);
        const auto reason = reject_reason(a);
        if (a.pose)
          tools::draw_text(
            preview,
            fmt::format("yaw {:.2f}  pitch {:.2f}  motion {:.3f} deg", a.pose->yaw * 180 / M_PI,
              a.pose->pitch * 180 / M_PI, a.pose->motion_deg * 180 / M_PI),
            {20, 30}, {0, 255, 255});
        tools::draw_text(
          preview, fmt::format("saved {}  auto {}", saved_count.load(), automatic ? "ON" : "off"), {20, 60},
          {255, 255, 255});
        tools::draw_text(
          preview, reason.empty() ? std::string("READY") : std::string("NOT READY"), {20, 90},
          reason.empty() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
        cv::imshow(window, preview);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
        if (key == 'a') automatic = !automatic;
        if (key == 's') {
          if (reason.empty())
            request_save(a);
          else
            fmt::print("[未保存] {}\n", reason);
        }
      }
    } catch (...) {
      stop = true;
      frame_ready.notify_all();
      save_ready.notify_all();
      detector_thread.join();
      saver_thread.join();
      throw;
    }
    stop = true;
    frame_ready.notify_all();
    save_ready.notify_all();
    detector_thread.join();
    saver_thread.join();
    fmt::print(
      "共保存 {} 张到 {}。求解：\n  {}\n", saved_count.load(), output.string(),
      use_serial ? fmt::format(
                     "build/calibrate_robotworld_handeye {0} --config-path={0}/board.yaml --output={0}/result.yaml "
                     "--preview=none",
                     output.string())
                 : fmt::format(
                     "build/calibrate_camera {0} --config-path={0}/board.yaml --camera-info={0}/camera_info.yaml "
                     "--output={0}/intrinsics.yaml --preview=none",
                     output.string()));
    return 0;
  } catch (const std::exception & error) {
    fmt::print(stderr, "[capture] {}\n", error.what());
    return 1;
  }
}
