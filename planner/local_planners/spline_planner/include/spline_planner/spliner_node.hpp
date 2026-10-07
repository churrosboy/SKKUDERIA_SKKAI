#ifndef SPLINE_PLANNER__SPLINER_NODE_HPP_
#define SPLINE_PLANNER__SPLINER_NODE_HPP_

#include <spline_planner/spliner_core.hpp>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/ot_wpnt_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace spline_planner
{

/// C++ port of spline_planner/spline_planner.py (node name "spliner_node").
class SplinerNode : public rclcpp::Node
{
public:
  SplinerNode();

private:
  double declareRanged(const std::string & name, double default_value, double from, double to);
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  void obsCallback(f110_msgs::msg::ObstacleArray::ConstSharedPtr msg);
  void stateCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void gbCallback(f110_msgs::msg::WpntArray::ConstSharedPtr msg);
  void gbScaledCallback(f110_msgs::msg::WpntArray::ConstSharedPtr msg);
  void loop();
  visualization_msgs::msg::Marker xyToPoint(double x, double y, bool opponent);

  std::unique_ptr<Spliner> spliner_;
  bool from_bag_{false};
  bool measuring_{false};
  bool state_print_{false};
  bool gb_print_{false};
  bool scaled_gb_print_{false};
  bool all_received_{false};
  unsigned viz_cycle_{0};
  builtin_interfaces::msg::Time last_switch_time_;

  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obs_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_subscription_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr gb_subscription_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr gb_scaled_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
  rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr evasion_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closest_obs_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr propagated_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace spline_planner

#endif  // SPLINE_PLANNER__SPLINER_NODE_HPP_
