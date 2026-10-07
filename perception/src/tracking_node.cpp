#include <perception/tracking_node.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

namespace perception
{

using std::placeholders::_1;

TrackingNode::TrackingNode()
: Node("tracking")
{
  measuring_ = declare_parameter<bool>("measure", false);
  from_bag_ = declare_parameter<bool>("from_bag", false);
  const TrackingParams params = declareParams();
  tracker_ = std::make_unique<Tracker>(params);
  current_stamp_ = get_clock()->now();

  RCLCPP_INFO(get_logger(), "Update rate: %.1f", params.rate_hz);

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&TrackingNode::parametersCallback, this, _1));

  // node clock (not wall clock) so that use_sim_time bag replays pace the loop like rclpy did
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / params.rate_hz),
    std::bind(&TrackingNode::loop, this));

  obstacles_subscription_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    "/perception/detection/raw_obstacles", 10,
    std::bind(&TrackingNode::obstacleCallback, this, _1));
  path_subscription_ = create_subscription<f110_msgs::msg::WpntArray>(
    "/global_waypoints", 10, std::bind(&TrackingNode::pathCallback, this, _1));
  frenet_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "/car_state/frenet/odom", 10, std::bind(&TrackingNode::carStateCallback, this, _1));
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "/car_state/odom", 10, std::bind(&TrackingNode::carStateGlobCallback, this, _1));
  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(), std::bind(&TrackingNode::scanCallback, this, _1));

  markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/static_dynamic_marker_pub", 5);
  obstacles_publisher_ = create_publisher<f110_msgs::msg::ObstacleArray>(
    "/perception/obstacles", 5);
  raw_opponent_publisher_ = create_publisher<f110_msgs::msg::ObstacleArray>(
    "/perception/raw_obstacles", 5);
  if (measuring_) {
    latency_publisher_ = create_publisher<std_msgs::msg::Float32>(
      "/perception/tracking/latency", 10);
  }

  RCLCPP_INFO(
    get_logger(), "[Tracking]: C++ tracking initialized (%.1f Hz, from_bag=%s)",
    params.rate_hz, from_bag_ ? "true" : "false");
}

double TrackingNode::declareNumber(const std::string & name, double default_value)
{
  // yaml may hold either 2 or 2.0 for the same knob -> accept both
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

TrackingParams TrackingNode::declareParams()
{
  TrackingParams p;
  // launch-time KF parameters (same names as tracking.py)
  p.rate_hz = declareNumber("rate", 40.0);
  p.p_vs = declareNumber("P_vs", 0.2);
  p.p_d = declareNumber("P_d", 0.02);
  p.p_vd = declareNumber("P_vd", 0.2);
  p.measurement_var_s = declareNumber("measurment_var_s", 0.002);
  p.measurement_var_d = declareNumber("measurment_var_d", 0.002);
  p.measurement_var_vs = declareNumber("measurment_var_vs", 0.2);
  p.measurement_var_vd = declareNumber("measurment_var_vd", 0.2);
  p.process_var_vs = declareNumber("process_var_vs", 2.0);
  p.process_var_vd = declareNumber("process_var_vd", 8.0);
  p.max_dist = declareNumber("max_dist", 0.5);
  p.var_pub = declareNumber("var_pub", 1.0);
  // dynamic-reconfigurable parameters
  p.ttl_dynamic = static_cast<int>(declareNumber("ttl_dynamic", 40.0));
  p.ratio_to_glob_path = declareNumber("ratio_to_glob_path", 0.6);
  p.ttl_static = static_cast<int>(declareNumber("ttl_static", 3.0));
  p.min_nb_meas = static_cast<int>(declareNumber("min_nb_meas", 6.0));
  p.min_std = declareNumber("min_std", 0.16);
  p.max_std = declareNumber("max_std", 0.2);
  p.dist_deletion = declareNumber("dist_deletion", 7.0);
  p.dist_infront = declareNumber("dist_infront", 8.0);
  p.vs_reset = declareNumber("vs_reset", 0.1);
  p.aggro_multiplier = declareNumber("aggro_multi", 2.0);
  p.debug_mode = declare_parameter<bool>("debug_mode", false);
  p.publish_static = declare_parameter<bool>("publish_static", true);
  p.no_memory_mode = declare_parameter<bool>("noMemoryMode", false);
  if (!(p.rate_hz > 0.0)) {
    throw std::invalid_argument("rate must be positive");
  }
  return p;
}

rcl_interfaces::msg::SetParametersResult TrackingNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  auto numeric = [](const rclcpp::Parameter & parameter) {
      return parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
             static_cast<double>(parameter.as_int()) : parameter.as_double();
    };
  TrackingParams & p = tracker_->params();
  try {
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "ttl_dynamic") {
        p.ttl_dynamic = static_cast<int>(numeric(parameter));
      } else if (name == "ratio_to_glob_path") {
        p.ratio_to_glob_path = numeric(parameter);
      } else if (name == "ttl_static") {
        p.ttl_static = static_cast<int>(numeric(parameter));
      } else if (name == "min_nb_meas") {
        p.min_nb_meas = static_cast<int>(numeric(parameter));
      } else if (name == "min_std") {
        p.min_std = numeric(parameter);
      } else if (name == "max_std") {
        p.max_std = numeric(parameter);
      } else if (name == "dist_deletion") {
        p.dist_deletion = numeric(parameter);
      } else if (name == "dist_infront") {
        p.dist_infront = numeric(parameter);
      } else if (name == "vs_reset") {
        p.vs_reset = numeric(parameter);
      } else if (name == "aggro_multi") {
        p.aggro_multiplier = numeric(parameter);
      } else if (name == "debug_mode") {
        p.debug_mode = parameter.as_bool();
      } else if (name == "publish_static") {
        p.publish_static = parameter.as_bool();
      } else if (name == "noMemoryMode") {
        p.no_memory_mode = parameter.as_bool();
      }
    }
  } catch (const std::exception & error) {
    result.successful = false;
    result.reason = error.what();
    return result;
  }
  RCLCPP_INFO(
    get_logger(),
    "[Tracking] Dynamic reconf triggered new tracking params: Tracking TTL: %d, "
    "Ratio to glob path: %.3f, ObstacleSD ttl, min_nb_meas, min_std, max_std: "
    "[%d, %d, %.3f, %.3f], dist_deletion: %.2f [m], dist_infront: %.2f [m], vs_reset: %.3f, "
    "aggro_multi: %.3f, Publish static obstacles: %s, no memory mode: %s",
    p.ttl_dynamic, p.ratio_to_glob_path, p.ttl_static, p.min_nb_meas, p.min_std, p.max_std,
    p.dist_deletion, p.dist_infront, p.vs_reset, p.aggro_multiplier,
    p.publish_static ? "true" : "false", p.no_memory_mode ? "true" : "false");
  return result;
}

void TrackingNode::obstacleCallback(f110_msgs::msg::ObstacleArray::ConstSharedPtr msg)
{
  tracker_->setMeasurements(msg->obstacles);
  current_stamp_ = msg->header.stamp;
}

void TrackingNode::pathCallback(f110_msgs::msg::WpntArray::ConstSharedPtr msg)
{
  if (tracker_->hasPath()) {
    return;
  }
  RCLCPP_INFO(get_logger(), "[Tracking] received global path");
  try {
    if (tracker_->setGlobalPath(msg->wpnts)) {
      RCLCPP_INFO(get_logger(), "[Tracking] initialized FrenetConverter object");
    } else {
      RCLCPP_WARN(get_logger(), "[Tracking] ignoring global path with fewer than two waypoints");
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "[Tracking] invalid global path: %s", error.what());
  }
}

void TrackingNode::carStateCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  tracker_->setCarS(msg->pose.pose.position.x);
}

void TrackingNode::carStateGlobCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const auto & q = msg->pose.pose.orientation;
  // yaw of tf_transformations.euler_from_quaternion (sxyz): atan2(M10, M00)
  const double yaw = std::atan2(2.0 * (q.x * q.y + q.w * q.z), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  tracker_->setCarPose(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
}

void TrackingNode::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  ScanData scan;
  scan.ranges = msg->ranges;
  scan.angle_min = msg->angle_min;
  scan.angle_max = msg->angle_max;
  scan.angle_increment = msg->angle_increment;
  tracker_->setScan(std::move(scan));
}

void TrackingNode::loop()
{
  const auto started = std::chrono::steady_clock::now();
  tracker_->step();
  if (measuring_) {
    std_msgs::msg::Float32 latency;
    latency.data = static_cast<float>(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
    latency_publisher_->publish(latency);
  }
  publishObstacles();
  publishMarkers();
}

void TrackingNode::publishObstacles()
{
  // python publishes (possibly empty) arrays every cycle, even before the path arrived
  f110_msgs::msg::ObstacleArray message;
  message.header.frame_id = "map";
  message.header.stamp = current_stamp_;
  std::vector<f110_msgs::msg::Obstacle> raw_opponent;
  tracker_->buildObstacleArrays(message.obstacles, raw_opponent);
  obstacles_publisher_->publish(message);
  message.obstacles = std::move(raw_opponent);
  raw_opponent_publisher_->publish(message);
}

void TrackingNode::publishMarkers()
{
  if (markers_publisher_->get_subscription_count() == 0) { return; }  // rviz-off: skip marker
  if (!tracker_->hasPath()) {
    return;
  }
  const auto & params = tracker_->params();
  const auto & converter = tracker_->converter();
  visualization_msgs::msg::MarkerArray markers;

  auto make_marker = [this](int id, double scale) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = current_stamp_;
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.scale.x = scale;
      marker.scale.y = scale;
      marker.scale.z = scale;
      marker.color.a = 0.5;
      marker.pose.orientation.w = 1.0;
      return marker;
    };

  for (const auto & tracked : tracker_->trackedObstacles()) {
    if (!params.publish_static || tracked.static_flag == StaticFlag::Dynamic) {
      continue;
    }
    auto marker = make_marker(tracked.id, tracked.is_in_front ? 0.5 : 0.25);
    frenet_conversion::GlobalPoint point;
    if (tracked.static_flag == StaticFlag::Unknown) {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      point = converter.getGlobalPoint(tracked.measurements_s.back(), tracked.measurements_d.back());
    } else {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      point = converter.getGlobalPoint(tracked.mean[0], tracked.mean[1]);
    }
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    markers.markers.push_back(marker);
  }

  const auto & opponent = tracker_->opponent();
  if (opponent.is_initialised) {
    auto marker = make_marker(opponent.id, opponent.P(0, 0) < params.var_pub ? 0.5 : 0.25);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    const auto point = converter.getGlobalPoint(
      pyMod(opponent.x[0], tracker_->trackLength()), opponent.x[2]);
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    markers.markers.push_back(marker);
  }

  visualization_msgs::msg::MarkerArray clear;
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  clear.markers.push_back(delete_all);
  markers_publisher_->publish(clear);
  markers_publisher_->publish(markers);
}

}  // namespace perception
