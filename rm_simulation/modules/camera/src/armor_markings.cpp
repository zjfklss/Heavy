#include "rm_sim/armor_markings.hpp"
#include "armor_markings_2026.hpp"
#include <stdexcept>

namespace rm_sim {
const ArmorMarking& armor_marking(std::string_view id) {
  for(const auto* marking:{&mark_1,&mark_2,&mark_3,&mark_4,&mark_sentry,&mark_outpost})
    if(marking->id==id)return *marking;
  throw std::invalid_argument("Unknown RM2026 armor marking");
}
const ArmorMarking& armor_marking(const TargetState& target) {
  return armor_marking(target.kind==TargetKind::outpost ? "outpost":target.armor_marking);
}
}
