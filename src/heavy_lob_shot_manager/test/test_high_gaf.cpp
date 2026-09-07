#include "heavy_lob_shot_manager/windmill.hpp"
#include "projectile_motion/gaf_projectile_solver.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace
{
// 与 ITL GafProjectileSolver 内 GRAVITY 一致
constexpr double kG = 9.7913;
constexpr double kV = 16.5;

void vacuumPitch(double horizontal, double height_diff, double speed, double gravity,
  double & low, double & high)
{
  const double v2 = speed * speed;
  const double disc = v2 * v2 - gravity * (gravity * horizontal * horizontal + 2.0 * height_diff * v2);
  ASSERT_GE(disc, 0.0);
  const double root = std::sqrt(disc);
  low = std::atan((v2 - root) / (gravity * horizontal));
  high = std::atan((v2 + root) / (gravity * horizontal));
}
}

TEST(HighGaf, Demo01GeometryPicksHighBranch)
{
  const double dx = 23.125 - 3.00;
  const double dy = 1.510 - 1.50;
  const double dz = 0.840 - 0.40;
  const double x = std::hypot(dx, dy);

  double vac_low = 0.0;
  double vac_high = 0.0;
  vacuumPitch(x, dz, kV, kG, vac_low, vac_high);
  EXPECT_GT(vac_high, 45.0 * M_PI / 180.0);
  EXPECT_LT(vac_low, 45.0 * M_PI / 180.0);
  EXPECT_GT(vac_high - vac_low, 10.0 * M_PI / 180.0);

  projectile_motion::GafProjectileSolver solver(kV, 0.0);
  double high = 0.0;
  double low = 0.0;
  ASSERT_TRUE(solver.solve_high(x, dz, high));
  ASSERT_TRUE(solver.solve(x, dz, low));
  EXPECT_NEAR(high, vac_high, 1e-6);
  EXPECT_LT(std::fabs(low - vac_low), std::fabs(low - vac_high));
}

TEST(HighGaf, Demo01LineMissesCenterWindmill)
{
  EXPECT_FALSE(heavy_lob_shot_manager::pointInVerticalCylinder(
    3.0, 1.5, 1.0, 14.5, 8.0, 1.7, 0.2, 2.8));
  EXPECT_FALSE(heavy_lob_shot_manager::pointInVerticalCylinder(
    14.5, 1.51, 4.0, 14.5, 8.0, 1.7, 0.2, 2.8));
  EXPECT_TRUE(heavy_lob_shot_manager::pointInVerticalCylinder(
    14.5, 8.0, 1.5, 14.5, 8.0, 1.7, 0.2, 2.8));
}

TEST(HighGaf, LosIterationWouldNotBeHigh)
{
  const double x = 20.125;
  const double y = 0.44;
  const double los = std::atan2(y, x);
  double vac_low = 0.0;
  double vac_high = 0.0;
  vacuumPitch(x, y, kV, kG, vac_low, vac_high);
  EXPECT_LT(std::fabs(los - vac_low), std::fabs(los - vac_high));

  projectile_motion::GafProjectileSolver solver(kV, 0.0);
  double itl_low = 0.0;
  ASSERT_TRUE(solver.solve(x, y, itl_low));
  EXPECT_LT(std::fabs(itl_low - vac_low), std::fabs(itl_low - vac_high));
}
