#include <gtest/gtest.h>

#include <frenet_conversion/frenet_converter.hpp>

#include <stdexcept>
#include <vector>

namespace
{

std::vector<f110_msgs::msg::Wpnt> straightWaypoints()
{
  std::vector<f110_msgs::msg::Wpnt> points(4);
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i].s_m = static_cast<double>(i);
    points[i].x_m = static_cast<double>(i);
    points[i].y_m = 0.0;
    points[i].psi_rad = 0.0;
  }
  return points;
}

}  // namespace

TEST(FrenetConverter, ConvertsPointAboveStraightTrack)
{
  frenet_conversion::FrenetConverter converter;
  converter.setGlobalTrajectory(straightWaypoints(), false);
  const auto result = converter.getFrenetPoint(1.25, 0.30, true);
  EXPECT_NEAR(result.s, 1.25, 1e-6);
  EXPECT_NEAR(result.d, 0.30, 1e-6);
}

TEST(FrenetConverter, WrapsClosedTrackSIntoTrackLength)
{
  frenet_conversion::FrenetConverter converter;
  converter.setGlobalTrajectory(straightWaypoints(), true);
  const auto result = converter.getFrenetPoint(3.10, 0.0, true);
  EXPECT_NEAR(result.s, 0.10, 1e-6);
}

TEST(FrenetConverter, ConvertsBackToGlobalCoordinates)
{
  frenet_conversion::FrenetConverter converter;
  converter.setGlobalTrajectory(straightWaypoints(), false);
  const auto result = converter.getGlobalPoint(1.50, -0.20);
  EXPECT_NEAR(result.x, 1.50, 1e-6);
  EXPECT_NEAR(result.y, -0.20, 1e-6);
}

TEST(FrenetConverter, RejectsUseBeforeTrajectory)
{
  frenet_conversion::FrenetConverter converter;
  EXPECT_THROW(
  {
    const auto result = converter.getFrenetPoint(0.0, 0.0, true);
    (void)result;
  },
    std::logic_error);
}
