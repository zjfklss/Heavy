#include "config.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace hero
{
YAML::Node validate_config(const std::filesystem::path & path)
{
  auto config = YAML::LoadFile(path.string());
  for (const auto & item : std::vector<std::pair<std::string, size_t>>{
         {"camera_matrix", 9}, {"distort_coeffs", 5}, {"R_camera2gimbal", 9},
         {"R_gimbal2imubody", 9}, {"t_camera2gimbal", 3}, {"camera_rpy", 3}}) {
    const auto values = config[item.first].as<std::vector<double>>();
    if (values.size() != item.second) throw std::runtime_error("Invalid matrix size: " + item.first);
    for (double value : values)
      if (!std::isfinite(value)) throw std::runtime_error("Non-finite matrix: " + item.first);
  }
  for (const auto & key : {"R_camera2gimbal", "R_gimbal2imubody"}) {
    const auto values = config[key].as<std::vector<double>>();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation(values.data());
    if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
        std::abs(rotation.determinant() - 1) > 1e-6)
      throw std::runtime_error(std::string("Invalid rotation: ") + key);
  }
  const auto color = config["enemy_color"].as<std::string>();
  if (color != "blue" && color != "red") throw std::runtime_error("Invalid enemy_color");
  if (config["yolo_name"].as<std::string>() != "yolov5")
    throw std::runtime_error("The imported multithread pipeline requires yolov5");
  for (const auto & key : {"yolov5_model_path", "classify_model"}) {
    auto model = std::filesystem::path(config[key].as<std::string>());
    if (model.is_relative()) model = std::filesystem::absolute(path).parent_path() / model;
    if (!std::filesystem::is_regular_file(model)) throw std::runtime_error("Missing model: " + model.string());
    config[key] = model.string();
    if (model.extension() == ".xml") {
      model.replace_extension(".bin");
      if (!std::filesystem::is_regular_file(model)) throw std::runtime_error("Missing weights: " + model.string());
    }
  }
  for (const auto & key : {"fps", "exposure_ms", "decision_timeout"}) {
    const double value = config[key].as<double>();
    if (!std::isfinite(value) || value <= 0) throw std::runtime_error(std::string("Invalid ") + key);
  }
  if (config["image_width"].as<int>() <= 0 || config["image_height"].as<int>() <= 0)
    throw std::runtime_error("Invalid image dimensions");
  const auto serial = config["serial"];
  if (serial["baud_rate"].as<int>() != 921600 && serial["baud_rate"].as<int>() != 115200)
    throw std::runtime_error("Supported serial baud rates: 921600, 115200");
  const double alpha = serial["bullet_speed_alpha"].as<double>();
  if (!std::isfinite(alpha) || alpha <= 0 || alpha > 1 || serial["timeout"].as<double>() <= 0)
    throw std::runtime_error("Invalid serial timing/smoothing");
  const double speed = serial["bullet_speed"].as<double>();
  if (!std::isfinite(speed) || speed <= 10 || speed >= 20) throw std::runtime_error("Invalid fallback bullet speed");
  for (const auto & key : {"shoot_delay", "mouse_to_fire", "k_air_resistance", "layer_height_threshold"}) {
    const double value = config[key].as<double>();
    if (!std::isfinite(value) || value < 0) throw std::runtime_error(std::string("Invalid ") + key);
  }
  return config;
}

std::string resolved_config(const std::filesystem::path & path)
{
  return YAML::Dump(validate_config(path));
}
}
