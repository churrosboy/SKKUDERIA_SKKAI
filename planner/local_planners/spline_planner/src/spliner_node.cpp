#include <spline_planner/spliner_node.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

namespace spline_planner
{

using std::placeholders::_1;

SplinerNode::SplinerNode()
: Node("spliner_node")
{
  from_bag_ = declare_parameter<bool>("from_bag", false);
  measuring_ = declare_parameter<bool>("measure", false);
  last_switch_time_ = get_clock()->now();

  SplinerParams p;
  // python declares the pre apexes as positive distances and stores them negated
  p.pre_apex_0 = -declareRanged("pre_apex_0", std::abs(p.pre_apex_0), 0.1, 8.0);
  p.pre_apex_1 = -declareRanged("pre_apex_1", std::abs(p.pre_apex_1), 0.1, 8.0);
  p.pre_apex_2 = -declareRanged("pre_apex_2", std::abs(p.pre_apex_2), 0.1, 8.0);
  p.post_apex_0 = declareRanged("post_apex_0", p.post_apex_0, 0.0, 8.0);
  p.post_apex_1 = declareRanged("post_apex_1", p.post_apex_1, 0.0, 8.0);
  p.post_apex_2 = declareRanged("post_apex_2", p.post_apex_2, 0.0, 8.0);
  p.post_apex_hold_m = declareRanged("post_apex_hold_m", p.post_apex_hold_m, 0.0, 8.0);
  p.obs_group_gap_m = declareRanged("obs_group_gap_m", p.obs_group_gap_m, 0.0, 8.0);
  p.evasion_dist = declareRanged("evasion_dist", p.evasion_dist, 0.1, 8.0);
  p.obs_traj_tresh = declareRanged("obs_traj_tresh", p.obs_traj_tresh, 0.1, 8.0);
  p.spline_bound_mindist = declareRanged("spline_bound_mindist", p.spline_bound_mindist, 0.1, 8.0);
  p.fixed_pred_time = declareRanged("fixed_pred_time", p.fixed_pred_time, 0.1, 8.0);
  // evasion cache (opt-in, default OFF). See EvasionCacheEntry.
  p.cache_enable = declare_parameter<bool>("cache_enable", p.cache_enable);
  p.cache_pos_tol_m = declareRanged("cache_pos_tol_m", p.cache_pos_tol_m, 0.0, 2.0);
  p.cache_vs_tol_mps = declareRanged("cache_vs_tol_mps", p.cache_vs_tol_mps, 0.0, 20.0);
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 1;
    range.to_value = 128;
    range.step = 1;
    descriptor.integer_range.push_back(range);
    p.cache_max_entries =
      static_cast<int>(declare_parameter<int>("cache_max_entries", p.cache_max_entries, descriptor));
  }

  spliner_ = std::make_unique<Spliner>(
    p, [this](LogLevel level, const std::string & message) {
      if (level == LogLevel::Warn) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", message.c_str());
      } else if (level == LogLevel::InfoThrottled) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", message.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
      }
    });

  obs_subscription_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    "/perception/obstacles", 10, std::bind(&SplinerNode::obsCallback, this, _1));
  state_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "/car_state/frenet/odom", 10, std::bind(&SplinerNode::stateCallback, this, _1));
  gb_subscription_ = create_subscription<f110_msgs::msg::WpntArray>(
    "/global_waypoints", 10, std::bind(&SplinerNode::gbCallback, this, _1));
  gb_scaled_subscription_ = create_subscription<f110_msgs::msg::WpntArray>(
    "/global_waypoints_scaled", 10, std::bind(&SplinerNode::gbScaledCallback, this, _1));

  markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planner/avoidance/markers", 10);
  evasion_publisher_ = create_publisher<f110_msgs::msg::OTWpntArray>(
    "/planner/avoidance/otwpnts", 10);
  closest_obs_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "/planner/avoidance/considered_OBS", 10);
  propagated_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
    "/planner/avoidance/propagated_obs", 10);
  if (measuring_) {
    latency_publisher_ = create_publisher<std_msgs::msg::Float32>(
      "/planner/avoidance/latency", 10);
  }

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&SplinerNode::parametersCallback, this, _1));

  RCLCPP_INFO(get_logger(), "Carstate Node waiting for Odometry messages...");
  // python blocks in wait_for_messages(); here the 20 Hz loop simply idles until ready
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / 20.0),
    std::bind(&SplinerNode::loop, this));
}

double SplinerNode::declareRanged(
  const std::string & name, double default_value, double from, double to)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = 0.001;
  descriptor.floating_point_range.push_back(range);
  return declare_parameter<double>(name, default_value, descriptor);
}

rcl_interfaces::msg::SetParametersResult SplinerNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  SplinerParams & p = spliner_->params();
  try {
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "pre_apex_0") {
        p.pre_apex_0 = -1.0 * parameter.as_double();
      } else if (name == "pre_apex_1") {
        p.pre_apex_1 = -1.0 * parameter.as_double();
      } else if (name == "pre_apex_2") {
        p.pre_apex_2 = -1.0 * parameter.as_double();
      } else if (name == "post_apex_0") {
        p.post_apex_0 = parameter.as_double();
      } else if (name == "post_apex_1") {
        p.post_apex_1 = parameter.as_double();
      } else if (name == "post_apex_2") {
        p.post_apex_2 = parameter.as_double();
      } else if (name == "post_apex_hold_m") {
        p.post_apex_hold_m = parameter.as_double();
      } else if (name == "obs_group_gap_m") {
        p.obs_group_gap_m = parameter.as_double();
      } else if (name == "evasion_dist") {
        p.evasion_dist = parameter.as_double();
      } else if (name == "obs_traj_tresh") {
        p.obs_traj_tresh = parameter.as_double();
      } else if (name == "spline_bound_mindist") {
        p.spline_bound_mindist = parameter.as_double();
      } else if (name == "fixed_pred_time") {
        p.fixed_pred_time = parameter.as_double();
      } else if (name == "cache_enable") {
        p.cache_enable = parameter.as_bool();
      } else if (name == "cache_pos_tol_m") {
        p.cache_pos_tol_m = parameter.as_double();
      } else if (name == "cache_vs_tol_mps") {
        p.cache_vs_tol_mps = parameter.as_double();
      } else if (name == "cache_max_entries") {
        p.cache_max_entries = static_cast<int>(parameter.as_int());
      }
    }
  } catch (const std::exception & error) {
    result.successful = false;
    result.reason = error.what();
    return result;
  }
  // every spline param changes knots / apex / veto -> cached evasions are stale
  spliner_->clearCache();
  RCLCPP_INFO(
    get_logger(),
    " Dynamic reconf triggered new spline params:\n pre apexes: [%g, %g, %g] [m],"
    " post apexes: [%g, %g, %g] [m],\n post apex hold: %g [m], obstacle group gap: %g [m],\n"
    " evasion apex distance: %g [m], obstacle trajectory treshold: %g [m],"
    " obstacle prediction constant time: %g [s],\n evasion cache: %s (pos tol %g [m],"
    " vs tol %g [m/s], max %d entries; cache cleared)",
    p.pre_apex_0, p.pre_apex_1, p.pre_apex_2, p.post_apex_0, p.post_apex_1, p.post_apex_2,
    p.post_apex_hold_m, p.obs_group_gap_m, p.evasion_dist, p.obs_traj_tresh, p.fixed_pred_time,
    p.cache_enable ? "ON" : "off", p.cache_pos_tol_m, p.cache_vs_tol_mps, p.cache_max_entries);
  return result;
}

void SplinerNode::obsCallback(f110_msgs::msg::ObstacleArray::ConstSharedPtr msg)
{
  spliner_->setObstacles(msg->obstacles);
}

void SplinerNode::stateCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  spliner_->setState(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->twist.twist.linear.x);
  if (!state_print_) {
    RCLCPP_INFO(get_logger(), "Received State message.");
    state_print_ = true;
  }
}

void SplinerNode::gbCallback(f110_msgs::msg::WpntArray::ConstSharedPtr msg)
{
  try {
    spliner_->setGlobalPath(msg->wpnts);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Invalid global path: %s", error.what());
    return;
  }
  if (!gb_print_) {
    RCLCPP_INFO(get_logger(), "Received Global Waypoints message.");
    gb_print_ = true;
  }
}

void SplinerNode::gbScaledCallback(f110_msgs::msg::WpntArray::ConstSharedPtr msg)
{
  spliner_->setScaledPath(msg->wpnts);
  if (!scaled_gb_print_) {
    RCLCPP_INFO(get_logger(), "Received Scaled Global Waypoints message.");
    scaled_gb_print_ = true;
  }
}

visualization_msgs::msg::Marker SplinerNode::xyToPoint(double x, double y, bool opponent)
{
  visualization_msgs::msg::Marker mrk;
  mrk.header.frame_id = "map";
  mrk.header.stamp = get_clock()->now();
  mrk.type = visualization_msgs::msg::Marker::SPHERE;
  mrk.scale.x = 0.5;
  mrk.scale.y = 0.5;
  mrk.scale.z = 0.5;
  mrk.color.a = 0.8;
  mrk.color.b = 0.65;
  mrk.color.r = opponent ? 1.0 : 0.0;
  mrk.color.g = 0.65;
  mrk.pose.position.x = x;
  mrk.pose.position.y = y;
  mrk.pose.position.z = 0.01;
  mrk.pose.orientation.w = 1.0;
  return mrk;
}

void SplinerNode::loop()
{
  if (!spliner_->ready()) {
    return;
  }
  if (!all_received_) {
    RCLCPP_INFO(get_logger(), "All required messages received. Continuing...");
    all_received_ = true;
  }
  const auto started = std::chrono::steady_clock::now();

  // rviz markers are cosmetic: build/publish only when subscribed, at most 5 Hz (20/4)
  viz_cycle_ += 1;
  const bool viz_on = markers_publisher_->get_subscription_count() > 0 && viz_cycle_ % 4 == 0;
  const bool want_propagated = propagated_publisher_->get_subscription_count() > 0;

  const SplinerOutput out = spliner_->step(viz_on, want_propagated);
  const auto now = get_clock()->now();

  for (const auto & p : out.propagated) {
    propagated_publisher_->publish(xyToPoint(p.x, p.y, true));
  }
  if (out.has_considered && closest_obs_publisher_->get_subscription_count() > 0) {
    closest_obs_publisher_->publish(xyToPoint(out.considered_x, out.considered_y, false));
  }

  f110_msgs::msg::OTWpntArray wpnts;
  visualization_msgs::msg::MarkerArray mrks;
  if (out.evaluated) {
    wpnts.header.stamp = now;
    wpnts.header.frame_id = "map";
    wpnts.wpnts = out.wpnts;
    if (!out.danger) {
      wpnts.ot_side = out.ot_side;
      wpnts.ot_line = out.ot_line;
      wpnts.side_switch = out.side_switch;
      wpnts.last_switch_time = last_switch_time_;
      if (out.switch_time_now) {
        last_switch_time_ = now;
      }
    } else {
      wpnts.side_switch = true;
      last_switch_time_ = now;
    }
    const double gb_vmax = spliner_->gbVmax();
    for (const auto & m : out.markers) {
      visualization_msgs::msg::Marker mrk;
      mrk.header.frame_id = "map";
      mrk.header.stamp = now;
      mrk.type = visualization_msgs::msg::Marker::CYLINDER;
      mrk.scale.x = 0.1;
      mrk.scale.y = 0.1;
      mrk.scale.z = m.v / gb_vmax;
      mrk.color.a = 1.0;
      mrk.color.b = 0.75;
      mrk.color.r = 0.75;
      if (from_bag_) {
        mrk.color.g = 0.75;
      }
      mrk.id = static_cast<int>(mrks.markers.size());
      mrk.pose.position.x = m.x;
      mrk.pose.position.y = m.y;
      mrk.pose.position.z = m.v / (gb_vmax / 2.0);
      mrk.pose.orientation.w = 1.0;
      mrks.markers.push_back(mrk);
    }
  } else if (out.delete_markers) {
    visualization_msgs::msg::Marker del;
    del.header.stamp = now;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    mrks.markers.push_back(del);
  }

  if (measuring_) {
    std_msgs::msg::Float32 latency;
    latency.data = static_cast<float>(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
    latency_publisher_->publish(latency);
  }
  evasion_publisher_->publish(wpnts);
  if (viz_on) {
    markers_publisher_->publish(mrks);
  }
}

}  // namespace spline_planner
