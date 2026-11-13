#include "camera.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>
#ifdef IO_WITH_HIKROBOT
#include "hikrobot/hikrobot.hpp"
#endif

namespace io
{
Camera::Camera(const std::string & config_path)
{
#ifdef IO_WITH_HIKROBOT
  const auto config = YAML::LoadFile(config_path);
  camera_ = std::make_unique<HikRobot>(config["exposure_ms"].as<double>(),
    config["gain"].as<double>(), "", config["fps"].as<double>(), HikRobotSettings::load(config));
#else
  throw std::runtime_error("Build with Hikrobot MVS to use a live camera");
#endif
}
void Camera::read(cv::Mat & image, std::chrono::steady_clock::time_point & stamp)
{
  camera_->read(image, stamp);
}
}
