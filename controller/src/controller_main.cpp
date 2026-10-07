#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "controller/controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<controller::ControllerNode>();
  node->init();  // throws on fatal misconfiguration (wrong mode, missing yaml)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
