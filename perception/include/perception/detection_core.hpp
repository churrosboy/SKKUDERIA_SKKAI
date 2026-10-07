#ifndef PERCEPTION__DETECTION_CORE_HPP_
#define PERCEPTION__DETECTION_CORE_HPP_

#include <cstddef>
#include <functional>
#include <vector>

namespace perception
{

struct ScanPoint
{
  double x;
  double y;
  double range;
  bool valid;
};

struct MapPoint
{
  double x;
  double y;
};

struct DetectedObstacle
{
  int id;
  double center_x;
  double center_y;
  double size;
  double theta;
};

struct DetectionConfig
{
  double lambda_rad{10.0 * 3.14159265358979323846 / 180.0};
  double sigma_m{0.03};
  double new_cluster_threshold_m{0.4};
  double min_obstacle_size_m{0.2};
  double max_obstacle_size_m{1.0};
  double min_two_points_distance_m{0.01};
  std::size_t min_points{10U};
};

using Cluster = std::vector<ScanPoint>;
using PointPredicate = std::function<bool (double, double)>;

[[nodiscard]] std::vector<Cluster> clusterScan(
  const std::vector<ScanPoint> & points,
  double angle_increment,
  const DetectionConfig & config,
  const PointPredicate & is_on_track);

[[nodiscard]] std::vector<DetectedObstacle> fitLShapes(
  const std::vector<Cluster> & clusters,
  const MapPoint & sensor_position,
  const DetectionConfig & config);

[[nodiscard]] std::vector<DetectedObstacle> filterAndNumberObstacles(
  const std::vector<DetectedObstacle> & obstacles,
  const DetectionConfig & config);

[[nodiscard]] double wrapPositive(double value, double period);

}  // namespace perception

#endif  // PERCEPTION__DETECTION_CORE_HPP_
