#ifndef IO__HIKROBOT_SETTINGS_HPP
#define IO__HIKROBOT_SETTINGS_HPP

#include <optional>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace io
{
// Missing fields retain the device settings used by the original camera profiles.
struct HikRobotSettings
{
  int image_width = 0;
  int image_height = 0;
  int offset_x = -1;
  int offset_y = -1;
  std::optional<bool> trigger_enable;
  std::optional<bool> frame_rate_enable;
  std::string serial;
  int image_node_num = 8;

  static HikRobotSettings load(const YAML::Node & yaml)
  {
    HikRobotSettings settings;
    settings.serial = yaml["camera_serial"].as<std::string>();
    settings.image_node_num = yaml["image_node_num"].as<int>();
    auto read_int = [&yaml](const char * key, int fallback, int minimum) {
      if (!yaml[key]) return fallback;
      const auto value = yaml[key].as<int>();
      if (value < minimum) throw std::invalid_argument(std::string("Invalid: ") + key);
      return value;
    };
    settings.image_width = read_int("image_width", 0, 1);
    settings.image_height = read_int("image_height", 0, 1);
    settings.offset_x = read_int("offset_x", -1, 0);
    settings.offset_y = read_int("offset_y", -1, 0);
    if (yaml["trigger_enable"]) settings.trigger_enable = yaml["trigger_enable"].as<bool>();
    if (yaml["frame_rate_enable"])
      settings.frame_rate_enable = yaml["frame_rate_enable"].as<bool>();
    return settings;
  }
};
}  // namespace io

#endif  // IO__HIKROBOT_SETTINGS_HPP
