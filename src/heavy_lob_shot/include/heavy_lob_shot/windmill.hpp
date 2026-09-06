#ifndef HEAVY_LOB_SHOT__WINDMILL_HPP_
#define HEAVY_LOB_SHOT__WINDMILL_HPP_

namespace heavy_lob_shot
{

// RMUC 中央能量机关 sweep 圆柱。产品规则，ITL 没有对应物。
inline bool pointInVerticalCylinder(
  double x, double y, double z,
  double cx, double cy, double radius,
  double z_min, double z_max)
{
  if (z < z_min || z > z_max) {
    return false;
  }
  const double dx = x - cx;
  const double dy = y - cy;
  return dx * dx + dy * dy <= radius * radius;
}

}  // namespace heavy_lob_shot

#endif  // HEAVY_LOB_SHOT__WINDMILL_HPP_
