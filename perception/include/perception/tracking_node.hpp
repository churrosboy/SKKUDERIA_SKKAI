#ifndef PERCEPTION__TRACKING_NODE_HPP_
#define PERCEPTION__TRACKING_NODE_HPP_

#include <perception/tracking_core.hpp>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace perception
{

/// C++ port of perception/tracking.py (node name "tracking").
class TrackingNode : public rclcpp::Node
{
public:
  TrackingNode();

private:
  double declareNumber(const std::string & name, double default_value);
  TrackingParams declareParams();

  void obstacleCallback(f110_msgs::msg::ObstacleArray::ConstSharedPtr msg);
  void pathCallback(f110_msgs::msg::WpntArray::ConstSharedPtr msg);
  void carStateCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void carStateGlobCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void loop();
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void publishObstacles();
  void publishMarkers();

  std::unique_ptr<Tracker> tracker_;
  bool measuring_{false};
  bool from_bag_{false};
  builtin_interfaces::msg::Time current_stamp_;

  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_subscription_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
  rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr raw_opponent_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace perception

#endif  // PERCEPTION__TRACKING_NODE_HPP_
