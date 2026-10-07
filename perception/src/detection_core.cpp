#include <perception/detection_core.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace perception
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double distance(const ScanPoint & lhs, const ScanPoint & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

struct Extents
{
  double min_first;
  double max_first;
  double min_second;
  double max_second;
};

Extents projectedExtents(const Cluster & cluster, double cosine, double sine)
{
  Extents extents{
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()};
  for (const auto & point : cluster) {
    const double first = point.x * cosine + point.y * sine;
    const double second = -point.x * sine + point.y * cosine;
    extents.min_first = std::min(extents.min_first, first);
    extents.max_first = std::max(extents.max_first, first);
    extents.min_second = std::min(extents.min_second, second);
    extents.max_second = std::max(extents.max_second, second);
  }
  return extents;
}

double candidateScore(
  const Cluster & cluster, double cosine, double sine,
  const Extents & extents, double minimum_distance)
{
  double first_to_max_norm = 0.0;
  double first_to_min_norm = 0.0;
  double second_to_max_norm = 0.0;
  double second_to_min_norm = 0.0;
  for (const auto & point : cluster) {
    const double first = point.x * cosine + point.y * sine;
    const double second = -point.x * sine + point.y * cosine;
    first_to_max_norm += std::pow(extents.max_first - first, 2);
    first_to_min_norm += std::pow(first - extents.min_first, 2);
    second_to_max_norm += std::pow(extents.max_second - second, 2);
    second_to_min_norm += std::pow(second - extents.min_second, 2);
  }

  const bool use_first_min = first_to_max_norm > first_to_min_norm;
  const bool use_second_min = second_to_max_norm > second_to_min_norm;
  double score = 0.0;
  for (const auto & point : cluster) {
    const double first = point.x * cosine + point.y * sine;
    const double second = -point.x * sine + point.y * cosine;
    const double first_distance = use_first_min ?
      first - extents.min_first : extents.max_first - first;
    const double second_distance = use_second_min ?
      second - extents.min_second : extents.max_second - second;
    score += 1.0 / std::max(
      std::min(first_distance, second_distance), minimum_distance);
  }
  return score;
}

}  // namespace

std::vector<Cluster> clusterScan(
  const std::vector<ScanPoint> & points,
  double angle_increment,
  const DetectionConfig & config,
  const PointPredicate & is_on_track)
{
  std::vector<Cluster> clusters;
  const double denominator = std::sin(config.lambda_rad - angle_increment);
  if (config.lambda_rad <= std::abs(angle_increment) || std::abs(denominator) < 1e-9) {
    throw std::invalid_argument("lambda_rad must be greater than angle_increment");
  }
  const double division_constant = std::sin(angle_increment) / denominator;

  for (const auto & point : points) {
    if (!point.valid || !std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.range) || !is_on_track(point.x, point.y))
    {
      continue;
    }
    if (clusters.empty()) {
      clusters.push_back({point});
      continue;
    }

    const double adaptive_threshold =
      point.range * division_constant + 3.0 * config.sigma_m;
    if (distance(point, clusters.back().back()) < adaptive_threshold) {
      clusters.back().push_back(point);
      continue;
    }

    auto closest = clusters.end();
    double closest_distance = std::numeric_limits<double>::infinity();
    for (auto cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
      const double candidate_distance = distance(point, cluster->back());
      if (candidate_distance < closest_distance) {
        closest_distance = candidate_distance;
        closest = cluster;
      }
    }
    if (closest != clusters.end() &&
      closest_distance < config.new_cluster_threshold_m)
    {
      Cluster reconnected = std::move(*closest);
      clusters.erase(closest);
      reconnected.push_back(point);
      clusters.push_back(std::move(reconnected));
    } else {
      clusters.push_back({point});
    }
  }

  clusters.erase(
    std::remove_if(
      clusters.begin(), clusters.end(),
      [&config](const Cluster & cluster) {return cluster.size() < config.min_points;}),
    clusters.end());
  return clusters;
}

std::vector<DetectedObstacle> fitLShapes(
  const std::vector<Cluster> & clusters,
  const MapPoint & sensor_position,
  const DetectionConfig & config)
{
  constexpr int kCandidateCount = 90;
  constexpr double kEndAngle = kPi / 2.0 - kPi / 180.0;
  std::vector<DetectedObstacle> obstacles;
  obstacles.reserve(clusters.size());

  for (const auto & cluster : clusters) {
    if (cluster.empty()) {
      continue;
    }
    double best_angle = 0.0;
    double best_score = -std::numeric_limits<double>::infinity();
    for (int candidate = 0; candidate < kCandidateCount; ++candidate) {
      const double angle = kEndAngle * candidate / (kCandidateCount - 1);
      const double cosine = std::cos(angle);
      const double sine = std::sin(angle);
      const auto extents = projectedExtents(cluster, cosine, sine);
      const double score = candidateScore(
        cluster, cosine, sine, extents, config.min_two_points_distance_m);
      if (score > best_score) {
        best_score = score;
        best_angle = angle;
      }
    }

    const double cosine = std::cos(best_angle);
    const double sine = std::sin(best_angle);
    const auto extents = projectedExtents(cluster, cosine, sine);
    const double sensor_first = sensor_position.x * cosine + sensor_position.y * sine;
    const double sensor_second = -sensor_position.x * sine + sensor_position.y * cosine;
    const std::array<MapPoint, 4> corners{{
      {extents.max_first, extents.max_second},
      {extents.max_first, extents.min_second},
      {extents.min_first, extents.max_second},
      {extents.min_first, extents.min_second}}};

    std::size_t closest_corner = 0U;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < corners.size(); ++index) {
      const double candidate_distance = std::hypot(
        corners[index].x - sensor_first, corners[index].y - sensor_second);
      if (candidate_distance < closest_distance) {
        closest_distance = candidate_distance;
        closest_corner = index;
      }
    }

    const double size = std::max(
      std::max(
        extents.max_first - extents.min_first,
        extents.max_second - extents.min_second),
      config.min_obstacle_size_m);
    double center_first = corners[closest_corner].x;
    double center_second = corners[closest_corner].y;
    center_first += (closest_corner < 2U ? -1.0 : 1.0) * size / 2.0;
    center_second += (closest_corner % 2U == 0U ? -1.0 : 1.0) * size / 2.0;

    obstacles.push_back(
      {
        -1,
        cosine * center_first - sine * center_second,
        sine * center_first + cosine * center_second,
        size,
        best_angle});
  }
  return obstacles;
}

std::vector<DetectedObstacle> filterAndNumberObstacles(
  const std::vector<DetectedObstacle> & obstacles,
  const DetectionConfig & config)
{
  std::vector<DetectedObstacle> filtered;
  filtered.reserve(obstacles.size());
  for (const auto & obstacle : obstacles) {
    if (obstacle.size <= config.max_obstacle_size_m) {
      auto numbered = obstacle;
      numbered.id = static_cast<int>(filtered.size());
      filtered.push_back(numbered);
    }
  }
  return filtered;
}

double wrapPositive(double value, double period)
{
  if (!(period > 0.0)) {
    throw std::invalid_argument("wrap period must be positive");
  }
  return std::fmod(std::fmod(value, period) + period, period);
}

}  // namespace perception
