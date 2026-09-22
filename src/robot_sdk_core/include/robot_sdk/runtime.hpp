#pragma once

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_sdk/config.hpp"

namespace robot_sdk {

/// 持有 Robot 实例的 ROS 2 节点、回调组和执行器。
///
/// 三个回调组编码了 SDK 的并发契约 —— 这正是客户自己写时最容易搞错的部分：
///   - control: 互斥(MutuallyExclusive)，把目标/命令处理串行化
///   - sensor : 可重入(Reentrant)，让高频感知流并发展开
///   - status : 互斥，把健康状态回调隔离在控制路径之外
/// 多线程执行器让这些组独立运行，因此**慢的感知订阅者永远无法阻塞导航目标**。
///
/// 核心心法：**回调组 = 线程语义**。
class RobotRuntime {
 public:
  /// 自建节点 + 自建多线程执行器。调用 spin() 即可驱动。
  explicit RobotRuntime(const RobotConfig& config);

  /// 挂到调用方已有的节点上。不创建执行器，由调用方负责 spin。
  RobotRuntime(const rclcpp::Node::SharedPtr& node, const RobotConfig& config);

  const rclcpp::Node::SharedPtr& node() const { return node_; }
  const RobotConfig& config() const { return config_; }

  const rclcpp::CallbackGroup::SharedPtr& controlGroup() const { return control_group_; }
  const rclcpp::CallbackGroup::SharedPtr& sensorGroup() const { return sensor_group_; }
  const rclcpp::CallbackGroup::SharedPtr& statusGroup() const { return status_group_; }

  /// 执行器是不是我自己建的？（寄生模式下为 false，此时 spin() 是空操作）
  bool ownsExecutor() const { return static_cast<bool>(executor_); }

  /// 最多驱动 timeout 时间的回调。
  void spinSome(std::chrono::milliseconds timeout);

  /// 一直驱动回调，直到 stop() 被调用。寄生模式下是空操作。
  void spin();

  void stop();

 private:
  void setup();  // 两个构造函数共用的初始化

  RobotConfig config_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_group_;
  rclcpp::CallbackGroup::SharedPtr status_group_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
};

}  // namespace robot_sdk
