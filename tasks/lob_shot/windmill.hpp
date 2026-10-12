#ifndef LOB_SHOT__WINDMILL_HPP_
#define LOB_SHOT__WINDMILL_HPP_

namespace lob_shot
{

// RMUC 中央能量机关扫掠圆柱（Heavy 产品规则）
inline bool point_in_cylinder(
  double x, double y, double z,
  double cx, double cy, double radius,
  double z_min, double z_max)
{
  if (z < z_min || z > z_max) return false;
  const double dx = x - cx;
  const double dy = y - cy;
  return dx * dx + dy * dy <= radius * radius;
}

}  // namespace lob_shot

#endif
