#ifndef PERCEPTION__DETECT_NODE_HPP_
#define PERCEPTION__DETECT_NODE_HPP_

#include <perception/detection_core.hpp>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <frenet_conversion/frenet_converter.hpp>
#include <grid_filter/grid_filter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace perception
{

class DetectNode : public rclcpp::Node
{
public:
  DetectNode();

private:
  double declareNumber(const std::string & name, double default_value);
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
  void pathCallback(f110_msgs::msg::WpntArray::ConstSharedPtr path);
  void timerCallback();
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void publishBoundary(const f110_msgs::msg::WpntArray & path);
  void publishBreakpoints(
    const std::vector<Cluster> & clusters,
    const builtin_interfaces::msg::Time & stamp);
  void publishObstacles(
    const std::vector<DetectedObstacle> & obstacles,
    const builtin_interfaces::msg::Time & stamp);
  void publishObstacleMarkers(
    const std::vector<DetectedObstacle> & obstacles,
    const builtin_interfaces::msg::Time & stamp);
  visualization_msgs::msg::MarkerArray deleteAllMarkers(
    const builtin_interfaces::msg::Time & stamp) const;

  DetectionConfig config_;
  double rate_hz_{40.0};
  double max_viewing_distance_m_{9.0};
  double min_viewing_distance_m_{0.15};
  double boundaries_inflation_m_{0.1};
  int filter_kernel_size_{7};
  double max_tf_staleness_s_{0.05};
  bool measuring_{false};
  bool from_bag_{false};

  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  f110_msgs::msg::WpntArray::ConstSharedPtr latest_path_;
  frenet_conversion::FrenetConverter frenet_converter_;
  grid_filter::GridFilter grid_filter_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr path_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr breakpoints_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundary_publisher_;
  rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace perception

#endif  // PERCEPTION__DETECT_NODE_HPP_
