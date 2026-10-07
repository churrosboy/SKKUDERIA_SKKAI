#ifndef FRENET_CONVERSION__FRENET_CONVERTER_HPP_
#define FRENET_CONVERSION__FRENET_CONVERTER_HPP_

#include <f110_msgs/msg/wpnt.hpp>

#include <vector>

namespace frenet_conversion
{

struct FrenetPoint
{
  double s;
  double d;
  int closest_index;
};

struct GlobalPoint
{
  double x;
  double y;
};

class FrenetConverter
{
public:
  void setGlobalTrajectory(
    const std::vector<f110_msgs::msg::Wpnt> & waypoints,
    bool is_closed_contour);

  [[nodiscard]] FrenetPoint getFrenetPoint(
    double x, double y, bool full_search = true) const;

  [[nodiscard]] GlobalPoint getGlobalPoint(double s, double d) const;

  [[nodiscard]] bool hasTrajectory() const noexcept;
  [[nodiscard]] double trackLength() const;

private:
  [[nodiscard]] int closestCartesianIndex(double x, double y) const;
  [[nodiscard]] int closestFrenetIndex(double s) const;

  std::vector<f110_msgs::msg::Wpnt> waypoints_;
  bool is_closed_contour_{false};
  double track_length_{0.0};
};

}  // namespace frenet_conversion

#endif  // FRENET_CONVERSION__FRENET_CONVERTER_HPP_
