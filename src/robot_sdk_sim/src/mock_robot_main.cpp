#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_sdk_sim/mock_robot_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 模拟机器人自己就是一个完整程序，所以用多线程执行器：
  // 动作服务端、定时器、服务端可以并行处理。
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<robot_sdk::sim::MockRobotNode>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
