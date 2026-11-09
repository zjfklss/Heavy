#pragma once
#include <string>
#include <vector>

namespace rm_sim::adapters {
// Protocol conventions only; this registry never constructs or executes an aiming algorithm.
struct Profile {
  std::string id, label;
  double pitch_sign=1; // external pitch = simulation pitch * pitch_sign
};
const std::vector<Profile>& profiles();
const Profile& profile(const std::string& id);
} // namespace rm_sim::adapters
