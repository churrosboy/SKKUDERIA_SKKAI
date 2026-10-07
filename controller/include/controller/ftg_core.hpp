#ifndef CONTROLLER__FTG_CORE_HPP_
#define CONTROLLER__FTG_CORE_HPP_

// C++ port of controller/ftg.py (FTG_Controller): pure logic, no ROS dependencies.
// Deviations: DEBUG markers not ported, NaN rays map to 0.0 (blocked), too-short scans return {0, 0}.

#include <utility>
#include <vector>

namespace controller
{

// Mirrors the FTG_Controller ctor args (ftg_* params in controller_manager.py)
struct FtgParams
{
  bool mapping{false};
  int safety_radius{100};       // [beams] bubble size at range discontinuities
  double max_lidar_dist{5.0};   // [m]
  double max_speed{4.0};        // [m/s]
  int range_offset{375};        // only consider scan[range_offset:-range_offset]
  double track_width{1.65};     // [m]
};

class FtgCore
{
public:
  explicit FtgCore(const FtgParams & params);

  void setParams(const FtgParams & params);
  void setVelocity(double velocity);  // set_vel: feeds the gap radius

  // process_lidar: one scan in, (speed, steering_angle) out.
  // Returns {0, 0} if the scan is unusable (too short for range_offset).
  std::pair<double, double> processLidar(const std::vector<float> & ranges);

private:
  std::vector<double> preprocessLidar(const std::vector<float> & ranges);
  std::vector<double> safetyBorder(const std::vector<double> & ranges) const;
  std::pair<int, int> findLargestGap(
    const std::vector<double> & ranges, double radius) const;
  std::pair<double, double> getBestRangePoint(const std::vector<double> & proc_ranges);
  double getRadius() const;

  FtgParams params_;
  double velocity_{0.0};
  double radians_per_elem_{0.0};
  int n_beams_{0};
};

}  // namespace controller

#endif  // CONTROLLER__FTG_CORE_HPP_
