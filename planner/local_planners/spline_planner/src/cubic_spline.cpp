#include <spline_planner/cubic_spline.hpp>

#include <algorithm>
#include <stdexcept>

namespace spline_planner
{

namespace
{

// Thomas algorithm for a tridiagonal system (lower, diag, upper, rhs) -> solution
std::vector<double> solveTridiagonal(
  std::vector<double> lower, std::vector<double> diag, std::vector<double> upper,
  std::vector<double> rhs)
{
  const std::size_t n = diag.size();
  for (std::size_t i = 1; i < n; ++i) {
    const double w = lower[i] / diag[i - 1];
    diag[i] -= w * upper[i - 1];
    rhs[i] -= w * rhs[i - 1];
  }
  std::vector<double> x(n);
  x[n - 1] = rhs[n - 1] / diag[n - 1];
  for (std::size_t i = n - 1; i-- > 0;) {
    x[i] = (rhs[i] - upper[i] * x[i + 1]) / diag[i];
  }
  return x;
}

}  // namespace

CubicSpline::CubicSpline(const std::vector<double> & x, const std::vector<double> & y)
: x_(x), y_(y)
{
  const std::size_t n = x.size();
  if (n < 2U || y.size() != n) {
    throw std::invalid_argument("CubicSpline needs >= 2 points and equal-size x/y");
  }
  for (std::size_t i = 1; i < n; ++i) {
    if (!(x[i] > x[i - 1])) {
      throw std::invalid_argument("CubicSpline x must be strictly increasing");
    }
  }
  std::vector<double> dx(n - 1), slope(n - 1);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    dx[i] = x[i + 1] - x[i];
    slope[i] = (y[i + 1] - y[i]) / dx[i];
  }

  std::vector<double> s(n);  // first derivatives at the knots (scipy CubicSpline formulation)
  if (n == 2U) {
    s[0] = slope[0];
    s[1] = slope[0];
  } else if (n == 3U) {
    // scipy: single parabola through the three points, y = y0 + p (x-x0) + q (x-x0)^2
    const double q = (slope[1] - slope[0]) / (dx[0] + dx[1]);
    const double p = slope[0] - q * dx[0];
    s[0] = p;
    s[1] = p + 2.0 * q * dx[0];
    s[2] = p + 2.0 * q * (dx[0] + dx[1]);
  } else {
    std::vector<double> lower(n, 0.0), diag(n, 0.0), upper(n, 0.0), rhs(n, 0.0);
    for (std::size_t i = 1; i + 1 < n; ++i) {
      lower[i] = dx[i];
      diag[i] = 2.0 * (dx[i - 1] + dx[i]);
      upper[i] = dx[i - 1];
      rhs[i] = 3.0 * (dx[i] * slope[i - 1] + dx[i - 1] * slope[i]);
    }
    // not-a-knot boundary rows (scipy CubicSpline)
    double d = x[2] - x[0];
    diag[0] = dx[1];
    upper[0] = d;
    rhs[0] = ((dx[0] + 2.0 * d) * dx[1] * slope[0] + dx[0] * dx[0] * slope[1]) / d;
    d = x[n - 1] - x[n - 3];
    diag[n - 1] = dx[n - 3];  // scipy: dx[-2]
    lower[n - 1] = d;
    rhs[n - 1] = (dx[n - 2] * dx[n - 2] * slope[n - 3] +
      (2.0 * d + dx[n - 2]) * dx[n - 3] * slope[n - 2]) / d;
    s = solveTridiagonal(lower, diag, upper, rhs);
  }

  b_.resize(n - 1);
  c_.resize(n - 1);
  d_.resize(n - 1);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double t = (s[i] + s[i + 1] - 2.0 * slope[i]) / dx[i];
    b_[i] = s[i];
    c_[i] = (slope[i] - s[i]) / dx[i] - t;
    d_[i] = t / dx[i];
  }
}

std::size_t CubicSpline::segment(double x) const
{
  // scipy: interval search with extrapolation on the end segments
  if (x <= x_.front()) {
    return 0;
  }
  if (x >= x_.back()) {
    return x_.size() - 2;
  }
  const auto it = std::upper_bound(x_.begin(), x_.end(), x);
  return static_cast<std::size_t>(std::distance(x_.begin(), it)) - 1;
}

double CubicSpline::evaluate(double x, int nu) const
{
  if (!valid()) {
    throw std::logic_error("CubicSpline not initialised");
  }
  const std::size_t i = segment(x);
  const double t = x - x_[i];
  if (nu == 0) {
    return y_[i] + t * (b_[i] + t * (c_[i] + t * d_[i]));
  }
  return b_[i] + t * (2.0 * c_[i] + 3.0 * t * d_[i]);
}

}  // namespace spline_planner
