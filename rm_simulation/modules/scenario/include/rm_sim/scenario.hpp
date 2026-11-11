#pragma once
#include "rm_sim/world.hpp"
#include <nlohmann/json.hpp>

namespace rm_sim::scenario {
using nlohmann::json;
void keys(const json& value,std::initializer_list<const char*> allowed);
Vec3 vec(const json& value);
json js(Vec3 value);
json js(Kinematics value);
TargetConfig target(const json& value);
uint64_t ticks(double time,double dt);
json frame(const Snapshot& snapshot);
} // namespace rm_sim::scenario
