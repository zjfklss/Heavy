#ifndef TOOLS__OV_DEVICE_HPP_
#define TOOLS__OV_DEVICE_HPP_

#include <openvino/openvino.hpp>
#include <string>

#include "tools/logger.hpp"

namespace tools
{

// yaml 写 GPU 但机子只有 CPU 时，compile_model 会直接抛；回退到第一个可用设备。
inline std::string resolve_ov_device(ov::Core & core, const std::string & want)
{
  const auto avail = core.get_available_devices();
  for (const auto & d : avail) {
    if (d == want || d.rfind(want + ".", 0) == 0) return d;
  }
  if (!avail.empty()) {
    std::string list;
    for (size_t i = 0; i < avail.size(); ++i) {
      if (i) list += ",";
      list += avail[i];
    }
    tools::logger()->warn(
      "[OV] device '{}' unavailable (have {}); fallback '{}'", want, list, avail.front());
    return avail.front();
  }
  tools::logger()->error("[OV] no inference device");
  return want;
}

}  // namespace tools

#endif
