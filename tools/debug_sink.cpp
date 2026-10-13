#include "debug_sink.hpp"

#include <fmt/chrono.h>
#include <fmt/core.h>

#include <cmath>
#include <cstring>
#include <filesystem>

#include "tools/live_config.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"

#ifdef HEAVY_FOXGLOVE
#include <foxglove/channel.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/websocket.hpp>
#endif

namespace tools
{
namespace
{
template <typename T>
void get_to(const YAML::Node & n, const char * key, T & v)
{
  if (n && n[key]) v = n[key].as<T>();
}

// 面板 key 支持 debug.image_hz 这种嵌套写法
YAML::Node lookup(const YAML::Node & n, const std::string & key)
{
  const auto dot = key.find('.');
  if (dot == std::string::npos) return n[key];
  const YAML::Node child = n[key.substr(0, dot)];
  if (!child) return child;
  return lookup(child, key.substr(dot + 1));
}

uint64_t wall_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
}
}  // namespace

DebugOptions DebugOptions::from_yaml(const YAML::Node & d)
{
  DebugOptions o;
  get_to(d, "foxglove", o.foxglove);
  get_to(d, "foxglove_port", o.foxglove_port);
  get_to(d, "image_hz", o.image_hz);
  get_to(d, "image_scale", o.image_scale);
  get_to(d, "jpeg_quality", o.jpeg_quality);
  get_to(d, "raw", o.raw);
  get_to(d, "imshow", o.imshow);
  get_to(d, "mcap", o.mcap);
  get_to(d, "plot", o.plot);
  get_to(d, "plot_host", o.plot_host);
  get_to(d, "plot_port", o.plot_port);
  return o;
}

#ifdef HEAVY_FOXGLOVE
namespace
{
std::optional<foxglove::Parameter> to_param(const std::string & name, const YAML::Node & n)
{
  if (!n) return std::nullopt;
  try {
    if (n.IsSequence()) {
      std::vector<double> v;
      for (const auto & e : n) v.push_back(e.as<double>());
      return foxglove::Parameter(name, v);
    }
    const auto & s = n.Scalar();
    if (s == "true" || s == "false") return foxglove::Parameter(name, s == "true");
    size_t used = 0;
    const double d = std::stod(s, &used);
    if (used == s.size()) return foxglove::Parameter(name, d);
  } catch (...) {
  }
  return foxglove::Parameter(name, std::string_view(n.Scalar()));
}

// 面板值 → yaml 字面量；整数值写成 "5"，否则 as<int> 的 key（min_detect_count 等）会解析失败
std::string to_yaml(const foxglove::ParameterValueView & v)
{
  auto val = v.value();
  if (auto d = std::get_if<double>(&val)) {
    if (std::floor(*d) == *d && std::abs(*d) < 1e15) return fmt::format("{:.0f}", *d);
    return fmt::format("{}", *d);
  }
  if (auto i = std::get_if<int64_t>(&val)) return std::to_string(*i);
  if (auto b = std::get_if<bool>(&val)) return *b ? "true" : "false";
  if (auto s = std::get_if<std::string_view>(&val)) return std::string(*s);
  if (auto a = std::get_if<foxglove::ParameterValueView::Array>(&val)) {
    std::string out = "[";
    for (size_t k = 0; k < a->size(); ++k) out += (k ? ", " : "") + to_yaml((*a)[k]);
    return out + "]";
  }
  return "";
}
}  // namespace

struct DebugSink::Impl
{
  std::optional<foxglove::WebSocketServer> server;
  std::optional<foxglove::messages::CompressedImageChannel> raw;
  std::optional<foxglove::messages::CompressedImageChannel> vis;
  std::optional<foxglove::RawChannel> debug;
  std::optional<foxglove::McapWriter> mcap;
  std::string mcap_path;

  std::vector<foxglove::Parameter> params(
    LiveConfig * live, const std::vector<std::string> & keys,
    const std::vector<std::string> & names) const
  {
    std::vector<foxglove::Parameter> out;
    if (!live) return out;
    const auto node = live->node();
    for (const auto & k : names.empty() ? keys : names) {
      if (auto p = to_param(k, lookup(node, k))) out.push_back(std::move(*p));
    }
    return out;
  }

  void log_image(foxglove::messages::CompressedImageChannel & ch, const cv::Mat & img, int quality)
  {
    std::vector<uchar> buf;
    cv::imencode(".jpg", img, buf, {cv::IMWRITE_JPEG_QUALITY, quality});
    foxglove::messages::CompressedImage msg;
    const auto ns = wall_ns();
    msg.timestamp = foxglove::messages::Timestamp{
      static_cast<uint32_t>(ns / 1000000000ULL), static_cast<uint32_t>(ns % 1000000000ULL)};
    msg.frame_id = "camera";
    msg.format = "jpeg";
    msg.data.resize(buf.size());
    std::memcpy(msg.data.data(), buf.data(), buf.size());
    ch.log(msg);
  }
};
#else
struct DebugSink::Impl
{
};
#endif

DebugSink::DebugSink(const DebugOptions & opt, LiveConfig * live, std::vector<std::string> param_keys)
: impl_(std::make_unique<Impl>()),
  live_(live),
  param_keys_(std::move(param_keys)),
  image_hz_(opt.image_hz),
  image_scale_(opt.image_scale),
  jpeg_quality_(opt.jpeg_quality),
  raw_(opt.raw),
  imshow_(opt.imshow),
  foxglove_(false)
{
  if (opt.plot) {
    plotter_ = std::make_unique<Plotter>(opt.plot_host, static_cast<uint16_t>(opt.plot_port));
    logger()->info("[Debug] PlotJuggler UDP {}:{}", opt.plot_host, opt.plot_port);
  }

#ifdef HEAVY_FOXGLOVE
  if (opt.foxglove) {
    foxglove::WebSocketServerOptions ws;
    ws.name = "heavy_standard";
    ws.host = "0.0.0.0";  // 笔记本经网线直连小电脑
    ws.port = static_cast<uint16_t>(opt.foxglove_port);
    if (live_) {
      ws.parameter_handler.onGet = [this](
                                     uint32_t, std::optional<std::string_view>,
                                     const std::vector<std::string_view> & names,
                                     foxglove::GetParametersResponder && responder) {
        std::vector<std::string> keys(names.begin(), names.end());
        std::move(responder).respond(impl_->params(live_, param_keys_, keys));
      };
      ws.parameter_handler.onSet = [this](
                                     uint32_t, std::optional<std::string_view>,
                                     const std::vector<foxglove::ParameterView> & params,
                                     foxglove::SetParametersResponder && responder) {
        std::vector<foxglove::Parameter> echo;
        for (const auto & p : params) {
          if (!p.hasValue()) continue;
          const std::string key(p.name());
          const auto value = to_yaml(*p.value());
          live_->set(key, value);
          logger()->info("[Debug] 面板改参 {} = {}", key, value);
          echo.push_back(p.clone());
        }
        std::move(responder).respond(std::move(echo));
      };
    }
    auto server = foxglove::WebSocketServer::create(std::move(ws));
    if (server.has_value()) {
      impl_->server.emplace(std::move(server.value()));
      static const std::string schema = R"({"type":"object"})";
      impl_->raw.emplace(foxglove::messages::CompressedImageChannel::create("/raw").value());
      impl_->vis.emplace(foxglove::messages::CompressedImageChannel::create("/vis").value());
      impl_->debug.emplace(
        foxglove::RawChannel::create(
          "/debug", "json",
          foxglove::Schema{
            "heavy.Debug", "jsonschema", reinterpret_cast<const std::byte *>(schema.data()),
            schema.size()})
          .value());
      foxglove_ = true;
      logger()->info(
        "[Debug] Foxglove ws://<本机IP>:{}（/raw /vis /debug + 参数面板）", opt.foxglove_port);
    } else {
      logger()->warn(
        "[Debug] Foxglove 端口 {} 启动失败：{}", opt.foxglove_port,
        foxglove::strerror(server.error()));
    }

    if (opt.mcap) {
      std::filesystem::create_directory("records");
      impl_->mcap_path =
        fmt::format("records/{:%Y-%m-%d_%H-%M-%S}.mcap", std::chrono::system_clock::now());
      foxglove::McapWriterOptions mo;
      mo.path = impl_->mcap_path;
      auto writer = foxglove::McapWriter::create(mo);
      if (writer.has_value()) {
        impl_->mcap.emplace(std::move(writer.value()));
        foxglove_ = true;
        logger()->info("[Debug] MCAP 录制 {}", impl_->mcap_path);
      }
    }
  }
#else
  if (opt.foxglove) logger()->warn("[Debug] 未编入 Foxglove SDK（tools/get_foxglove_sdk.sh），只剩 PlotJuggler");
#endif

  if (foxglove_ || imshow_) thread_ = std::thread(&DebugSink::loop, this);
}

DebugSink::~DebugSink()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    quit_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
#ifdef HEAVY_FOXGLOVE
  if (impl_->mcap) impl_->mcap->close();
  if (impl_->server) impl_->server->stop();
#endif
}

void DebugSink::reconfigure(const YAML::Node & d)
{
  auto o = DebugOptions::from_yaml(d);
  image_hz_ = o.image_hz;
  image_scale_ = o.image_scale;
  jpeg_quality_ = o.jpeg_quality;
  raw_ = o.raw;
}

void DebugSink::publish_params()
{
#ifdef HEAVY_FOXGLOVE
  if (impl_->server && live_) impl_->server->publishParameterValues(impl_->params(live_, param_keys_, {}));
#endif
}

void DebugSink::plot(const nlohmann::json & data)
{
  if (plotter_) plotter_->plot(data);
#ifdef HEAVY_FOXGLOVE
  if (impl_->debug) {
    const auto s = data.dump();
    impl_->debug->log(reinterpret_cast<const std::byte *>(s.data()), s.size(), wall_ns());
  }
#endif
}

bool DebugSink::want_image() const
{
  if (!foxglove_ && !imshow_) return false;
  const double hz = image_hz_;
  if (hz <= 0) return false;
  return std::chrono::steady_clock::now() - last_image_ >=
         std::chrono::duration<double>(1.0 / hz);
}

void DebugSink::image(const cv::Mat & raw, Overlay ov)
{
  if (raw.empty() || !want_image()) return;
  last_image_ = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.emplace(raw, std::move(ov));  // 只留最新一帧；Mat 共享像素，不拷贝
  }
  cv_.notify_one();
}

void DebugSink::loop()
{
  while (true) {
    std::pair<cv::Mat, Overlay> job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return quit_ || pending_.has_value(); });
      if (quit_) return;
      job = std::move(*pending_);
      pending_.reset();
    }

    const double s = image_scale_;
    cv::Mat small;
    cv::resize(job.first, small, {}, s, s, cv::INTER_LINEAR);
    cv::Mat vis = small.clone();
    draw(vis, job.second, s);

#ifdef HEAVY_FOXGLOVE
    const int q = jpeg_quality_;
    if (impl_->raw && raw_) impl_->log_image(*impl_->raw, small, q);
    if (impl_->vis) impl_->log_image(*impl_->vis, vis, q);
#endif
    if (imshow_) {
      cv::imshow("raw", small);
      cv::imshow("vis", vis);
      cv::waitKey(1);
    }
  }
}

void DebugSink::draw(cv::Mat & img, const Overlay & ov, double s) const
{
  const auto font = cv::FONT_HERSHEY_SIMPLEX;
  if (ov.center) {
    cv::drawMarker(img, *ov.center * s, {255, 128, 0}, cv::MARKER_CROSS, 20, 1);
  }
  for (const auto & p : ov.polys) {
    std::vector<cv::Point> pts;
    for (const auto & q : p.pts) pts.emplace_back(q * s);
    cv::polylines(img, pts, true, p.color, 2);
  }
  for (const auto & t : ov.texts) {
    cv::putText(img, t.s, t.at * s, font, 0.45, t.color, 1, cv::LINE_AA);
  }
  for (size_t i = 0; i < ov.header.size(); ++i) {
    const cv::Point at(8, 22 + 22 * static_cast<int>(i));
    cv::putText(img, ov.header[i], at, font, 0.6, {0, 0, 0}, 4, cv::LINE_AA);
    cv::putText(img, ov.header[i], at, font, 0.6, {0, 255, 0}, 1, cv::LINE_AA);
  }
}

}  // namespace tools
