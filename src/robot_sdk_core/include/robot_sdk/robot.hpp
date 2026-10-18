#pragma once

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_sdk/config.hpp"
#include "robot_sdk/motion_api.hpp"
#include "robot_sdk/navigation_api.hpp"
#include "robot_sdk/perception_api.hpp"
#include "robot_sdk/status_api.hpp"
#include "robot_sdk/transform.hpp"

namespace robot_sdk {

/// 实现细节：节点 + 回调组 + 执行器的持有者。**故意只前向声明** ——
/// 它是 SDK 内部管道，客户不需要知道它存在（完整定义只在 robot.cpp 里）。
class RobotRuntime;

/// Robot SDK 的入口：四个能力 API（导航/运动/感知/状态）装进一个对象，并持有 ROS 2 管道。
/// 两种构造对应两种宿主关系：`Robot(config)` 自建节点与执行器（调 spin() 驱动）；
/// `Robot(node, config)` 挂到已有节点上、由调用方 spin。并发契约见 detail/runtime.hpp。
class Robot {
 public:
  /// 自建节点 + 多线程执行器 —— 调 `spin()` 就转起来了。
  explicit Robot(const RobotConfig& config = RobotConfig{});

  /// 挂到已有节点上，由调用方继续 spin（SDK 不抢线程）。
  Robot(const rclcpp::Node::SharedPtr& node, const RobotConfig& config = RobotConfig{});

  NavigationAPI& navigation() { return *navigation_; }
  MotionAPI& motion() { return *motion_; }
  PerceptionAPI& perception() { return *perception_; }
  StatusAPI& status() { return *status_; }
  TransformManager& tf() { return *transform_; }

  // 这三个访问器要碰 RobotRuntime，而它在这里是不完整类型 —— 所以定义在 .cpp 里。
  const RobotConfig& config() const;
  const rclcpp::Node::SharedPtr& node() const;
  bool ownsExecutor() const;

  /// 阻塞式 spin 循环。**寄生模式下是 no-op**（节点由调用方 spin）。
  void spin();

  /// 最多驱动 `timeout` 时间的回调。寄生模式下是 no-op。
  void spinOnce(std::chrono::milliseconds timeout);

  /// 让 `spin()` 返回。
  void stop();

 private:
  /// **唯一的装配点。** 上面两个公开构造都**委托**到这里 ——
  /// 委托构造（delegating constructor）是 C++ 消除重复装配的惯用法，
  /// 不需要额外的 helper 函数。
  explicit Robot(std::shared_ptr<RobotRuntime> runtime);

  std::shared_ptr<RobotRuntime> runtime_;
  std::shared_ptr<TransformManager> transform_;
  std::unique_ptr<NavigationAPI> navigation_;
  std::unique_ptr<MotionAPI> motion_;
  std::unique_ptr<PerceptionAPI> perception_;
  std::unique_ptr<StatusAPI> status_;
};

}  // namespace robot_sdk
