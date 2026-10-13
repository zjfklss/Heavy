#ifndef TOOLS__DEBUG_SINK_HPP
#define TOOLS__DEBUG_SINK_HPP

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace tools
{
class LiveConfig;
class Plotter;

// 一帧要叠的东西。主线程只算坐标（全分辨率像素），真正的缩放/画线/JPEG 在调试线程做。
struct Overlay
{
  struct Poly
  {
    std::vector<cv::Point2f> pts;
    cv::Scalar color;
  };
  struct Text
  {
    std::string s;
    cv::Point2f at;
    cv::Scalar color;
  };

  std::vector<std::string> header;  // 左上角逐行，对应旧 final_img 的 Latency 叠字
  std::vector<Poly> polys;
  std::vector<Text> texts;
  std::optional<cv::Point2f> center;  // 光心十字

  void poly(std::vector<cv::Point2f> pts, cv::Scalar color) { polys.push_back({std::move(pts), color}); }
  void text(std::string s, cv::Point2f at, cv::Scalar color) { texts.push_back({std::move(s), at, color}); }
};

// yaml `debug:` 段
struct DebugOptions
{
  bool foxglove = true;
  int foxglove_port = 8765;
  double image_hz = 15;      // 图像推送上限；曲线每帧都推
  double image_scale = 0.5;  // 推图前缩放，省带宽和 JPEG 时间
  int jpeg_quality = 75;
  bool raw = true;  // 是否推处理前原图 /raw
  bool imshow = false;  // 本机窗口，仅 Remmina 兜底
  bool mcap = false;    // 录 records/*.mcap，Foxglove 直接打开回看
  bool plot = true;     // PlotJuggler UDP
  std::string plot_host = "127.0.0.1";
  int plot_port = 9870;

  static DebugOptions from_yaml(const YAML::Node & debug);
};

// 调试输出全在这里，主循环只付出一次 json dump + 一次 try_push。
// 输出：Foxglove WebSocket（/raw /vis 图像、/debug 曲线、参数面板）+ PlotJuggler UDP + 可选 imshow/MCAP。
class DebugSink
{
public:
  // param_keys：Foxglove 参数面板可见/可改的 yaml key（支持 a.b）
  DebugSink(const DebugOptions & opt, LiveConfig * live, std::vector<std::string> param_keys);
  ~DebugSink();

  void reconfigure(const YAML::Node & debug);  // 只改节流/缩放/画质，端口等需重启
  void publish_params();  // 磁盘 yaml 热加载后把新值推给参数面板
  void plot(const nlohmann::json & data);
  bool want_image() const;  // 节流：false 时主线程连叠图坐标都不用算
  void image(const cv::Mat & raw, Overlay ov);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::unique_ptr<Plotter> plotter_;
  LiveConfig * live_;
  std::vector<std::string> param_keys_;

  std::atomic<double> image_hz_;
  std::atomic<double> image_scale_;
  std::atomic<int> jpeg_quality_;
  std::atomic<bool> raw_;
  bool imshow_;
  bool foxglove_;

  std::chrono::steady_clock::time_point last_image_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<std::pair<cv::Mat, Overlay>> pending_;
  bool quit_ = false;
  std::thread thread_;

  void loop();
  void draw(cv::Mat & img, const Overlay & ov, double s) const;
};

}  // namespace tools

#endif  // TOOLS__DEBUG_SINK_HPP
