#ifndef SPLINE_PLANNER__PY_FRENET_CONVERTER_HPP_
#define SPLINE_PLANNER__PY_FRENET_CONVERTER_HPP_

#include <spline_planner/cubic_spline.hpp>

#include <vector>

namespace spline_planner
{

/// Faithful port of frenet_conversion/frenet_converter.py get_cartesian():
/// cubic splines x(s), y(s) over the CUMULATIVE EUCLIDEAN distance of the
/// waypoints (not wpnt.s_m), heading from the spline derivative at s % length.
/// Used instead of the C++ frenet_conversion lib so the evasion x/y are identical
/// to what the python spliner produced.
class PyFrenetConverter
{
public:
  PyFrenetConverter() = default;
  PyFrenetConverter(const std::vector<double> & x, const std::vector<double> & y);

  struct Point
  {
    double x;
    double y;
  };
  Point getCartesian(double s, double d) const;
  double racelineLength() const {return raceline_length_;}
  bool valid() const {return spline_x_.valid();}

private:
  CubicSpline spline_x_;
  CubicSpline spline_y_;
  double raceline_length_{0.0};
};

}  // namespace spline_planner

#endif  // SPLINE_PLANNER__PY_FRENET_CONVERTER_HPP_
