#include "rm_sim/adapters/profile.hpp"
#include <stdexcept>
namespace rm_sim::adapters {
Profile generic_profile();
Profile sp_vision_profile();
const std::vector<Profile>& profiles() {
  static const std::vector<Profile> entries{generic_profile(),sp_vision_profile()};
  return entries;
}
const Profile& profile(const std::string& id) {
  for(const auto& entry:profiles()) if(entry.id==id) return entry;
  throw std::invalid_argument("Unknown external adapter profile: "+id);
}
}
