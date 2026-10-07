#include <frenet_conversion/frenet_converter.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace frenet_conversion
{

namespace
{

double wrapPositive(double value, double period)
{
  return std::fmod(std::fmod(value, period) + period, period);
}

}  // namespace

void FrenetConverter::setGlobalTrajectory(
  const std::vector<f110_msgs::msg::Wpnt> & waypoints,
  bool is_closed_contour)
{
  if (waypoints.size() < 2U) {
    throw std::invalid_argument("Frenet trajectory requires at least two waypoints");
  }
  if (!(waypoints.back().s_m > 0.0)) {
    throw std::invalid_argument("Frenet trajectory length must be positive");
  }

  waypoints_ = waypoints;
  is_closed_contour_ = is_closed_contour;
  track_length_ = waypoints_.back().s_m;
}

FrenetPoint FrenetConverter::getFrenetPoint(
  double x, double y, bool /*full_search*/) const
{
  if (waypoints_.empty()) {
    throw std::logic_error("No global trajectory configured");
  }

  const int index = closestCartesianIndex(x, y);
  const auto & waypoint = waypoints_.at(static_cast<std::size_t>(index));
  const double dx = x - waypoint.x_m;
  const double dy = y - waypoint.y_m;
  double s = waypoint.s_m + dx * std::cos(waypoint.psi_rad) +
    dy * std::sin(waypoint.psi_rad);
  if (is_closed_contour_) {
    s = wrapPositive(s, track_length_);
  }
  const double d = -dx * std::sin(waypoint.psi_rad) +
    dy * std::cos(waypoint.psi_rad);
  return {s, d, index};
}

GlobalPoint FrenetConverter::getGlobalPoint(double s, double d) const
{
  if (waypoints_.empty()) {
    throw std::logic_error("No global trajectory configured");
  }
  if (is_closed_contour_) {
    s = wrapPositive(s, track_length_);
  }

  const int index = closestFrenetIndex(s);
  const auto & waypoint = waypoints_.at(static_cast<std::size_t>(index));
  const double ds = s - waypoint.s_m;
  return {
    waypoint.x_m + ds * std::cos(waypoint.psi_rad) - d * std::sin(waypoint.psi_rad),
    waypoint.y_m + ds * std::sin(waypoint.psi_rad) + d * std::cos(waypoint.psi_rad)};
}

bool FrenetConverter::hasTrajectory() const noexcept
{
  return !waypoints_.empty();
}

double FrenetConverter::trackLength() const
{
  if (waypoints_.empty()) {
    throw std::logic_error("No global trajectory configured");
  }
  return track_length_;
}

int FrenetConverter::closestCartesianIndex(double x, double y) const
{
  double best_distance = std::numeric_limits<double>::infinity();
  int best_index = 0;
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    const double dx = x - waypoints_[index].x_m;
    const double dy = y - waypoints_[index].y_m;
    const double squared_distance = dx * dx + dy * dy;
    if (squared_distance < best_distance) {
      best_distance = squared_distance;
      best_index = static_cast<int>(index);
    }
  }
  return best_index;
}

int FrenetConverter::closestFrenetIndex(double s) const
{
  auto closest = std::min_element(
    waypoints_.begin(), waypoints_.end(),
    [s](const auto & lhs, const auto & rhs) {
      return std::abs(lhs.s_m - s) < std::abs(rhs.s_m - s);
    });
  return static_cast<int>(std::distance(waypoints_.begin(), closest));
}

}  // namespace frenet_conversion
