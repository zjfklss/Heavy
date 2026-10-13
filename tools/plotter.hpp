#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <netinet/in.h>  // sockaddr_in

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools
{
// UDP 发 JSON 给 PlotJuggler（UDP Server，端口 9870）。heavy_standard 经 DebugSink 每帧调一次。
class Plotter
{
public:
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);

  ~Plotter();

  void plot(const nlohmann::json & json);

private:
  int socket_;
  sockaddr_in destination_;
  std::mutex mutex_;
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP