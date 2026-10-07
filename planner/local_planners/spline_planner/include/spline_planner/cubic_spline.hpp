#ifndef SPLINE_PLANNER__CUBIC_SPLINE_HPP_
#define SPLINE_PLANNER__CUBIC_SPLINE_HPP_

#include <cstddef>
#include <vector>

namespace spline_planner
{

/// Not-a-knot cubic spline (== scipy.interpolate.CubicSpline default and
/// InterpolatedUnivariateSpline(k=3) for n >= 4). n == 3 -> parabola, n == 2 -> line,
/// matching scipy.CubicSpline. Extrapolates with the end polynomials (scipy default).
class CubicSpline
{
public:
  CubicSpline() = default;
  CubicSpline(const std::vector<double> & x, const std::vector<double> & y);

  double operator()(double x) const {return evaluate(x, 0);}
  double derivative(double x) const {return evaluate(x, 1);}
  double evaluate(double x, int nu) const;
  bool valid() const {return x_.size() >= 2U;}
  double xMin() const {return x_.front();}
  double xMax() const {return x_.back();}

private:
  std::size_t segment(double x) const;

  std::vector<double> x_;
  std::vector<double> y_;
  // per-segment coefficients: y = a + b t + c t^2 + d t^3, t = x - x_i
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
};

}  // namespace spline_planner

#endif  // SPLINE_PLANNER__CUBIC_SPLINE_HPP_
