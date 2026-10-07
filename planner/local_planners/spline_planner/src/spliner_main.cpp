#include <spline_planner/spliner_node.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<spline_planner::SplinerNode>());
  rclcpp::shutdown();
  return 0;
}
