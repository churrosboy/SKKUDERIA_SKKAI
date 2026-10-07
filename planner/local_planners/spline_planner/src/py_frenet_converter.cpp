#include <spline_planner/py_frenet_converter.hpp>

#include <cmath>
#include <stdexcept>

namespace spline_planner
{

namespace
{
double pyMod(double value, double modulus)
{
  const double result = std::fmod(value, modulus);
  if (result != 0.0 && ((result < 0.0) != (modulus < 0.0))) {
    return result + modulus;
  }
  return result;
}
}  // namespace

PyFrenetConverter::PyFrenetConverter(
  const std::vector<double> & x, const std::vector<double> & y)
{
  if (x.size() < 2U || x.size() != y.size()) {
    throw std::invalid_argument("PyFrenetConverter needs >= 2 waypoints");
  }
  std::vector<double> s(x.size(), 0.0);
  for (std::size_t i = 1; i < x.size(); ++i) {
    s[i] = s[i - 1] + std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
  }
  spline_x_ = CubicSpline(s, x);
  spline_y_ = CubicSpline(s, y);
  raceline_length_ = s.back();
}

PyFrenetConverter::Point PyFrenetConverter::getCartesian(double s, double d) const
{
  double x = spline_x_(s);
  double y = spline_y_(s);
  const double sm = pyMod(s, raceline_length_);
  const double psi = std::atan2(spline_y_.derivative(sm), spline_x_.derivative(sm));
  x += d * std::cos(psi + M_PI / 2.0);
  y += d * std::sin(psi + M_PI / 2.0);
  return {x, y};
}

}  // namespace spline_planner
