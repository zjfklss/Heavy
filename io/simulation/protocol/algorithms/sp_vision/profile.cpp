#include "rm_sim/adapters/profile.hpp"
namespace rm_sim::adapters {
// SP uses radians, yaw left-positive, and pitch up-negative. No SP code is linked here.
Profile sp_vision_profile() { return {"sp_vision","SP Vision（抬头为负）",-1}; }
}
