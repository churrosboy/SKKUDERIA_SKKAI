// C++ port of controller/ftg.py -- see ftg_core.hpp for the deviation list.
// Line references are into ftg.py.

#include "controller/ftg_core.hpp"

#include <algorithm>
#include <cmath>

namespace controller
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int kPreprocessConvSize = 3;                 // PREPROCESS_CONV_SIZE
constexpr double kStraightsSteeringAngle = kPi / 18.0;  // 10 deg
constexpr double kMildCurveAngle = kPi / 6.0;           // 30 deg
constexpr double kUltraStraightsAngle = kPi / 60.0;     // 3 deg
constexpr double kSpeedScale = 0.6;                     // "scale" (:44)

}  // namespace

FtgCore::FtgCore(const FtgParams & params)
: params_(params)
{
}

void FtgCore::setParams(const FtgParams & params)
{
  params_ = params;
}

void FtgCore::setVelocity(double velocity)
{
  velocity_ = velocity;
}

std::vector<double> FtgCore::preprocessLidar(const std::vector<float> & ranges)
{
  // _preprocess_lidar (:72-97)
  const int n = static_cast<int>(ranges.size());
  radians_per_elem_ = (1.5 * kPi) / static_cast<double>(n);
  n_beams_ = n;

  // drop the beams behind us: ranges[range_offset:-range_offset]
  const int lo = params_.range_offset;
  const int hi = n - params_.range_offset;
  std::vector<double> sliced;
  sliced.reserve(hi - lo);
  for (int i = lo; i < hi; ++i) {
    const double v = static_cast<double>(ranges[i]);
    // NaN -> 0 (= blocked); python keeps NaN, which binarizes to blocked too
    sliced.push_back(std::isnan(v) ? 0.0 : v);
  }

  // np.convolve(x, ones(3)/3, 'valid'): out[i] = mean(x[i..i+2]), length m-2
  const int m = static_cast<int>(sliced.size());
  std::vector<double> proc(m - kPreprocessConvSize + 1);
  for (int i = 0; i < static_cast<int>(proc.size()); ++i) {
    double sum = 0.0;
    for (int j = 0; j < kPreprocessConvSize; ++j) {
      sum += sliced[i + j];
    }
    // clip to [0, MAX_LIDAR_DIST] (inf rays end up at max_lidar_dist)
    proc[i] = std::min(std::max(sum / kPreprocessConvSize, 0.0), params_.max_lidar_dist);
  }

  // reverse: lidar is right to left
  std::reverse(proc.begin(), proc.end());
  return proc;
}

std::vector<double> FtgCore::safetyBorder(const std::vector<double> & ranges) const
{
  // _safety_border (:297-326). Both passes intentionally READ the original
  // array while WRITING the filtered copy, exactly like the python.
  std::vector<double> filtered = ranges;
  const int len = static_cast<int>(ranges.size());
  const int sr = params_.safety_radius;
  int i = 0;
  while (i < len - 1) {
    if (ranges[i + 1] - ranges[i] > 0.5) {
      for (int j = 0; j < sr; ++j) {
        if (i + j < len) {
          filtered[i + j] = ranges[i];
        }
      }
      i += sr - 2;
    }
    i += 1;
  }
  // in the other direction
  i = len - 1;
  while (i > 0) {
    if (ranges[i - 1] - ranges[i] > 0.5) {
      for (int j = 0; j < sr; ++j) {
        if (i - j >= 0) {
          filtered[i - j] = ranges[i];
        }
      }
      i = i - sr + 2;
    }
    i -= 1;
  }
  return filtered;
}

std::pair<int, int> FtgCore::findLargestGap(
  const std::vector<double> & ranges, double radius) const
{
  // _find_largest_gap (:243-276)
  const int n = static_cast<int>(ranges.size());
  std::vector<int> bin_ranges(n);
  for (int i = 0; i < n; ++i) {
    bin_ranges[i] = ranges[i] >= radius ? 1 : 0;
  }

  // bin_diffs = |diff(bin)| with both ends forced to 1
  std::vector<int> bin_diffs(n - 1);
  for (int i = 0; i < n - 1; ++i) {
    bin_diffs[i] = std::abs(bin_ranges[i + 1] - bin_ranges[i]);
  }
  bin_diffs.front() = 1;
  bin_diffs.back() = 1;

  std::vector<int> diff_idxs;
  for (int i = 0; i < n - 1; ++i) {
    if (bin_diffs[i] != 0) {
      diff_idxs.push_back(i);
    }
  }
  if (diff_idxs.size() < 2) {
    return {0, 0};  // degenerate scan; python would index-error here
  }

  // score each segment: width if it is a "high" (open) segment, else 0;
  // argmax takes the FIRST maximum like np.argmax
  int best_left = diff_idxs[0];
  int best_width = 0;
  for (std::size_t k = 0; k + 1 < diff_idxs.size(); ++k) {
    const int low = diff_idxs[k];
    const int high = diff_idxs[k + 1];
    double sum = 0.0;
    for (int j = low; j < high; ++j) {
      sum += bin_ranges[j];
    }
    const bool is_high = (sum / static_cast<double>(high - low)) > 0.5;
    const int score = is_high ? (high - low) : 0;
    if (score > best_width) {
      best_width = score;
      best_left = low;
    }
  }
  return {best_left, best_left + best_width};
}

double FtgCore::getRadius() const
{
  // _get_radius (:278-286): empirically chosen, grows with speed
  return std::min(
    5.0, params_.track_width / 2.0 + 2.0 * (velocity_ / params_.max_speed));
}

std::pair<double, double> FtgCore::getBestRangePoint(
  const std::vector<double> & proc_ranges)
{
  // _get_best_range_point (:115-140); DEBUG markers not ported
  const double radius = getRadius();
  auto [gap_left, gap_right] = findLargestGap(proc_ranges, radius);

  // 45deg worth of beams (n_beams/6) recenters straight-ahead to 0 steering
  const int center_correction =
    static_cast<int>(std::lround(n_beams_ / 6.0));
  gap_left += params_.range_offset - center_correction;
  gap_right += params_.range_offset - center_correction;
  const int gap_middle = (gap_right + gap_left) / 2;

  const double best_y = std::cos(gap_middle * radians_per_elem_) * radius;
  const double best_x = std::sin(gap_middle * radians_per_elem_) * radius;
  return {best_x, best_y};
}

std::pair<double, double> FtgCore::processLidar(const std::vector<float> & ranges)
{
  // process_lidar (:186-241)
  // fail safe instead of crashing when the scan cannot survive the slicing
  const int n = static_cast<int>(ranges.size());
  if (n - 2 * params_.range_offset - (kPreprocessConvSize - 1) < 3) {
    return {0.0, 0.0};
  }

  std::vector<double> proc_ranges = preprocessLidar(ranges);
  proc_ranges = safetyBorder(proc_ranges);

  auto [best_x, best_y] = getBestRangePoint(proc_ranges);

  // _get_steer_angle (:99-113)
  const double steering_angle =
    std::min(std::max(std::atan2(best_y, best_x), -0.4), 0.4);

  double speed;
  if (params_.mapping) {
    speed = 1.5;
  } else if (std::abs(steering_angle) > kMildCurveAngle) {
    speed = 0.3 * params_.max_speed * kSpeedScale;   // CORNERS_SPEED
  } else if (std::abs(steering_angle) > kStraightsSteeringAngle) {
    speed = 0.45 * params_.max_speed * kSpeedScale;  // MILD_CORNERS_SPEED
  } else if (std::abs(steering_angle) > kUltraStraightsAngle) {
    speed = 0.8 * params_.max_speed * kSpeedScale;   // STRAIGHTS_SPEED
  } else {
    speed = params_.max_speed * kSpeedScale;         // ULTRASTRAIGHTS_SPEED
  }
  return {speed, steering_angle};
}

}  // namespace controller
