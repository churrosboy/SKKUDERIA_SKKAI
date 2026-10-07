#include <perception/detect_node.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

namespace perception
{

using std::placeholders::_1;

DetectNode::DetectNode()
: Node("detect")
{
  measuring_ = declare_parameter<bool>("measure", false);
  from_bag_ = declare_parameter<bool>("from_bag", false);
  rate_hz_ = declareNumber("rate", 40.0);
  config_.lambda_rad = declareNumber("lambda", 10.0) * M_PI / 180.0;
  config_.sigma_m = declareNumber("sigma", 0.03);
  config_.min_two_points_distance_m = declareNumber("min_2_points_dist", 0.01);
  config_.min_points = static_cast<std::size_t>(declareNumber("min_obs_size", 10.0));
  config_.min_obstacle_size_m = declareNumber("min_obs_size_m", 0.2);
  config_.max_obstacle_size_m = declareNumber("max_obs_size", 1.0);
  config_.new_cluster_threshold_m = declareNumber("new_cluster_threshold_m", 0.4);
  max_viewing_distance_m_ = declareNumber("max_viewing_distance", 9.0);
  min_viewing_distance_m_ = declareNumber("min_viewing_distance", 0.15);
  boundaries_inflation_m_ = declareNumber("boundaries_inflation", 0.1);
  filter_kernel_size_ = static_cast<int>(declareNumber("filter_kernel_size", 7.0));
  max_tf_staleness_s_ = declareNumber("max_tf_staleness", 0.05);
  const std::string map_name = declare_parameter<std::string>(
    "map_name", "hangar_1905_v0");

  grid_filter_.setErosionKernelSize(filter_kernel_size_);
  const auto map_yaml = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("stack_master")) /
    "maps" / map_name / (map_name + ".yaml");
  if (!grid_filter_.loadMap(map_yaml)) {
    throw std::runtime_error("Failed to load detection map: " + map_yaml.string());
  }

  breakpoints_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/breakpoints_markers", 5);
  auto boundary_qos = rclcpp::QoS(1).transient_local();
  boundary_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "/perception/detect_bound", boundary_qos);
  obstacles_publisher_ = create_publisher<f110_msgs::msg::ObstacleArray>(
    "/perception/detection/raw_obstacles", 5);
  markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/obstacles_markers_new", 5);
  if (measuring_) {
    latency_publisher_ = create_publisher<std_msgs::msg::Float32>(
      "/perception/detection/latency_ms", 5);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(), std::bind(&DetectNode::scanCallback, this, _1));
  path_subscription_ = create_subscription<f110_msgs::msg::WpntArray>(
    "/global_waypoints", 10, std::bind(&DetectNode::pathCallback, this, _1));
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&DetectNode::parametersCallback, this, _1));

  if (!(rate_hz_ > 0.0)) {
    throw std::invalid_argument("rate must be positive");
  }
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_hz_),
    std::bind(&DetectNode::timerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "C++ LiDAR detector ready at %.1f Hz (single-threaded executor, map=%s, from_bag=%s)",
    rate_hz_, map_name.c_str(), from_bag_ ? "true" : "false");
}

double DetectNode::declareNumber(const std::string & name, double default_value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  declare_parameter(name, rclcpp::ParameterValue(default_value), descriptor);
  const auto parameter = get_parameter(name);
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(parameter.as_int());
  }
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return parameter.as_double();
  }
  throw std::invalid_argument(name + " must be numeric");
}

void DetectNode::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  latest_scan_ = std::move(scan);
}

void DetectNode::pathCallback(f110_msgs::msg::WpntArray::ConstSharedPtr path)
{
  if (path->wpnts.size() < 2U) {
    RCLCPP_WARN(get_logger(), "Ignoring global path with fewer than two waypoints");
    return;
  }
  try {
    frenet_converter_.setGlobalTrajectory(path->wpnts, true);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Invalid global path: %s", error.what());
    return;
  }
  latest_path_ = std::move(path);
  publishBoundary(*latest_path_);
}

void DetectNode::timerCallback()
{
  if (!latest_scan_ || !frenet_converter_.hasTrajectory()) {
    return;
  }
  const auto scan = latest_scan_;
  const auto started = std::chrono::steady_clock::now();

  geometry_msgs::msg::TransformStamped transform_message;
  try {
    transform_message = tf_buffer_->lookupTransform(
      "map", scan->header.frame_id, rclcpp::Time(scan->header.stamp),
      rclcpp::Duration::from_seconds(0.0));
  } catch (const tf2::TransformException & error) {
    // Exact-stamp lookup can lose the race against localization; fall back to the
    // latest available transform, bounded by max_tf_staleness.
    try {
      transform_message = tf_buffer_->lookupTransform(
        "map", scan->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & fallback_error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LiDAR transform unavailable: %s", fallback_error.what());
      return;
    }
    const double staleness_s =
      (rclcpp::Time(scan->header.stamp) -
      rclcpp::Time(transform_message.header.stamp)).seconds();
    if (staleness_s > max_tf_staleness_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LiDAR transform too stale for fallback (%.0f ms > %.0f ms): %s",
        staleness_s * 1000.0, max_tf_staleness_s_ * 1000.0, error.what());
      return;
    }
  }

  tf2::Transform laser_to_map;
  tf2::fromMsg(transform_message.transform, laser_to_map);
  std::vector<ScanPoint> points;
  points.reserve(scan->ranges.size());
  for (std::size_t index = 0; index < scan->ranges.size(); ++index) {
    const double range = scan->ranges[index];
    // Drop returns inside min_viewing_distance so the car's own body parts
    // (mounts, cables) near the lidar are not clustered as obstacles.
    const bool valid = std::isfinite(range) && range >= scan->range_min &&
      range >= min_viewing_distance_m_ &&
      range <= scan->range_max && range <= max_viewing_distance_m_;
    if (!valid) {
      points.push_back({0.0, 0.0, range, false});
      continue;
    }
    const double angle = scan->angle_min + index * scan->angle_increment;
    const tf2::Vector3 map_point = laser_to_map * tf2::Vector3(
      range * std::cos(angle), range * std::sin(angle), 0.0);
    points.push_back({map_point.x(), map_point.y(), range, true});
  }

  const auto clusters = clusterScan(
    points, scan->angle_increment, config_,
    [this](double x, double y) {return grid_filter_.isPointInside(x, y);});
  const auto fitted = fitLShapes(
    clusters,
    {transform_message.transform.translation.x, transform_message.transform.translation.y},
    config_);
  const auto obstacles = filterAndNumberObstacles(fitted, config_);

  if (!obstacles.empty()) {
    const double scan_age_ms =
      (now() - rclcpp::Time(scan->header.stamp)).seconds() * 1000.0;
    std::string detail;
    for (const auto & obstacle : obstacles) {
      char buffer[96];
      std::snprintf(
        buffer, sizeof(buffer), " [id %d: x=%.2f y=%.2f size=%.2fm]",
        obstacle.id, obstacle.center_x, obstacle.center_y, obstacle.size);
      detail += buffer;
    }
  }

  publishBreakpoints(clusters, scan->header.stamp);
  publishObstacles(obstacles, scan->header.stamp);
  publishObstacleMarkers(obstacles, scan->header.stamp);

  if (latency_publisher_) {
    const auto elapsed = std::chrono::steady_clock::now() - started;
    std_msgs::msg::Float32 latency;
    latency.data = std::chrono::duration<float, std::milli>(elapsed).count();
    latency_publisher_->publish(latency);
  }
}

rcl_interfaces::msg::SetParametersResult DetectNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  auto numeric = [](const rclcpp::Parameter & parameter) {
      return parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
             static_cast<double>(parameter.as_int()) : parameter.as_double();
    };
  try {
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "lambda") {
        config_.lambda_rad = numeric(parameter) * M_PI / 180.0;
      } else if (name == "sigma") {
        config_.sigma_m = numeric(parameter);
      } else if (name == "min_2_points_dist") {
        config_.min_two_points_distance_m = numeric(parameter);
      } else if (name == "min_obs_size") {
        config_.min_points = static_cast<std::size_t>(numeric(parameter));
      } else if (name == "min_obs_size_m") {
        config_.min_obstacle_size_m = numeric(parameter);
      } else if (name == "max_obs_size") {
        config_.max_obstacle_size_m = numeric(parameter);
      } else if (name == "new_cluster_threshold_m") {
        config_.new_cluster_threshold_m = numeric(parameter);
      } else if (name == "max_viewing_distance") {
        max_viewing_distance_m_ = numeric(parameter);
      } else if (name == "min_viewing_distance") {
        min_viewing_distance_m_ = numeric(parameter);
      } else if (name == "boundaries_inflation") {
        boundaries_inflation_m_ = numeric(parameter);
        if (latest_path_) {
          publishBoundary(*latest_path_);
        }
      } else if (name == "filter_kernel_size") {
        filter_kernel_size_ = static_cast<int>(numeric(parameter));
        grid_filter_.setErosionKernelSize(filter_kernel_size_);
      } else if (name == "max_tf_staleness") {
        max_tf_staleness_s_ = numeric(parameter);
      }
    }
  } catch (const std::exception & error) {
    result.successful = false;
    result.reason = error.what();
  }
  return result;
}

void DetectNode::publishBoundary(const f110_msgs::msg::WpntArray & path)
{
  if (boundary_publisher_->get_subscription_count() == 0) { return; }  // rviz-off: skip marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = now();
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.a = 1.0F;
  marker.color.r = 1.0F;
  marker.points.reserve(path.wpnts.size() * 2U);
  for (const auto & waypoint : path.wpnts) {
    const auto right = frenet_converter_.getGlobalPoint(
      waypoint.s_m, -waypoint.d_right + boundaries_inflation_m_);
    const auto left = frenet_converter_.getGlobalPoint(
      waypoint.s_m, waypoint.d_left - boundaries_inflation_m_);
    geometry_msgs::msg::Point point;
    point.x = right.x;
    point.y = right.y;
    marker.points.push_back(point);
    point.x = left.x;
    point.y = left.y;
    marker.points.push_back(point);
  }
  boundary_publisher_->publish(marker);
}

void DetectNode::publishBreakpoints(
  const std::vector<Cluster> & clusters,
  const builtin_interfaces::msg::Time & stamp)
{
  if (breakpoints_publisher_->get_subscription_count() == 0) { return; }  // rviz-off: skip marker
  breakpoints_publisher_->publish(deleteAllMarkers(stamp));
  visualization_msgs::msg::MarkerArray markers;
  for (std::size_t index = 0; index < clusters.size(); ++index) {
    if (clusters[index].empty()) {
      continue;
    }
    for (int endpoint = 0; endpoint < 2; ++endpoint) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = stamp;
      marker.id = static_cast<int>(index * 10U) + endpoint * 2;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
      marker.color.a = 0.5F;
      marker.color.g = 1.0F;
      marker.color.b = clusters.empty() ? 0.0F :
        static_cast<float>(index) / static_cast<float>(clusters.size());
      const auto & point = endpoint == 0 ? clusters[index].front() : clusters[index].back();
      marker.pose.position.x = point.x;
      marker.pose.position.y = point.y;
      markers.markers.push_back(marker);
    }
  }
  breakpoints_publisher_->publish(markers);
}

void DetectNode::publishObstacles(
  const std::vector<DetectedObstacle> & obstacles,
  const builtin_interfaces::msg::Time & stamp)
{
  f110_msgs::msg::ObstacleArray message;
  message.header.frame_id = "map";
  message.header.stamp = stamp;
  const double track_length = frenet_converter_.trackLength();
  for (const auto & obstacle : obstacles) {
    const auto frenet = frenet_converter_.getFrenetPoint(
      obstacle.center_x, obstacle.center_y, true);
    const double half_size = obstacle.size / 2.0;
    f110_msgs::msg::Obstacle output;
    output.id = obstacle.id;
    output.s_start = wrapPositive(frenet.s - half_size, track_length);
    output.s_end = wrapPositive(frenet.s + half_size, track_length);
    output.d_left = frenet.d + half_size;
    output.d_right = frenet.d - half_size;
    output.s_center = frenet.s;
    output.d_center = frenet.d;
    output.size = obstacle.size;
    output.is_visible = true;
    message.obstacles.push_back(output);
  }
  obstacles_publisher_->publish(message);
}

void DetectNode::publishObstacleMarkers(
  const std::vector<DetectedObstacle> & obstacles,
  const builtin_interfaces::msg::Time & stamp)
{
  if (markers_publisher_->get_subscription_count() == 0) { return; }  // rviz-off: skip marker
  markers_publisher_->publish(deleteAllMarkers(stamp));
  visualization_msgs::msg::MarkerArray markers;
  for (const auto & obstacle : obstacles) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.id = obstacle.id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = obstacle.size;
    marker.color.a = 0.8F;
    marker.color.r = 1.0F;
    marker.pose.position.x = obstacle.center_x;
    marker.pose.position.y = obstacle.center_y;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, obstacle.theta);
    marker.pose.orientation = tf2::toMsg(orientation);
    markers.markers.push_back(marker);
  }
  markers_publisher_->publish(markers);
}

visualization_msgs::msg::MarkerArray DetectNode::deleteAllMarkers(
  const builtin_interfaces::msg::Time & stamp) const
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(marker);
  return markers;
}

}  // namespace perception
