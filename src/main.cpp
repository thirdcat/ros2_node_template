#include "ros2_node_template/ros2_node_template.h"

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<NodeName>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
