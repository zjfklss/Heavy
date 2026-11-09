#pragma once
// Read-only adapters from existing hero outputs to the diagnostic dashboard. No algorithm state changes here.
#include <list>

#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_outpost/shooter.hpp"
#include "tools/jlcv_show/dashboard.hpp"

inline void publish_hero_estimate(
  tools::jlcv_show::Channel * display, const std::list<auto_aim::Target> & targets,
  const auto_outpost::AimPoint & aim, const cv::Mat & image)
{
  if (!display) return;
  std::vector<tools::jlcv_show::ArmorPose> plates;
  std::string name;
  if (!targets.empty()) {
    const auto & target = targets.front();
    name = auto_aim::ARMOR_NAMES.at(target.name);
    for (const auto & xyza : target.armor_xyza_list())
      plates.push_back(
        {xyza, target.armor_type == auto_aim::big ? .23 : .135, .125,
         (target.name == auto_aim::outpost ? -15 : 15) * 3.14159265358979323846 / 180});
  }
  std::optional<Eigen::Vector3d> point;
  if (aim.valid) point = aim.xyza.head<3>();
  display->estimate(std::move(plates), std::move(point), std::move(name), image);
}
