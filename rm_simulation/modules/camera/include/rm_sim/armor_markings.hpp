#pragma once
#include "rm_sim/world.hpp"
#include <cstddef>
#include <string_view>

namespace rm_sim {
struct MarkingRect { int left,top,right,bottom; };
struct ArmorMarking {
  std::string_view id;
  const unsigned char* pixels;
  const MarkingRect* rectangles;
  size_t rectangle_count;
  static constexpr int width=96,height=128;
  static constexpr double physical_width=.09,physical_height=.12;
  bool ink(int x,int y) const { return pixels[y*(width/8)+x/8] & (1<<(x%8)); }
};
// RM2026 reference glyphs, shared by synthetic camera and UI; not algorithm-side labels.
const ArmorMarking& armor_marking(std::string_view id);
const ArmorMarking& armor_marking(const TargetState& target);
}
