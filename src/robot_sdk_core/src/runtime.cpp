#include "robot_sdk/detail/runtime.hpp"

#include <string>

namespace robot_sdk {

RobotRuntime::RobotRuntime(const RobotConfig& config) : config_(config) {
  rclcpp::NodeOptions options;
  // use_sim_time 必须在**节点构造时**通过 NodeOptions 注入 ——
  // 它决定了节点订阅哪个时间源，事后设置就来不及了。
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", config_.use_sim_time)});

  // 名字 + 命名空间，两个正交的参数（rclcpp::Node(name, namespace_, options)）。
  // 合法性交给 rcl 判 —— 它才是 ROS 名字语法的所有者。SDK 不自己定第二套规则、
  // 也不替调用者修正写错的配置。参数不合法时这里抛 rclcpp 的异常，Python 侧在
  // 语言边界上把它翻成 RobotSdkError（见 robot_sdk_py 的绑定）。
  node_ = std::make_shared<rclcpp::Node>(config_.node_name, config_.node_namespace, options);

  // 关键：多线程执行器。默认的单线程执行器会把所有回调串成一条线。
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(node_);
  setup();
}

RobotRuntime::RobotRuntime(const rclcpp::Node::SharedPtr& node, const RobotConfig& config)
    : config_(config), node_(node) {
  // 注意：这里**不建**执行器。节点已经属于调用方，由调用方 spin。
  setup();
}

void RobotRuntime::setup() {
  // 三个组 = 三种线程语义。见 runtime.hpp 的类注释。
  // 控制路径：串行 —— 同一时刻只处理一个目标，避免两个导航目标互相打架。
  control_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // 感知路径：并发 —— 五个传感器订阅彼此独立、又是高频，串行会形成瓶颈。
  sensor_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // 状态路径：单独一组并串行 —— 目的是"隔离"，不让状态回调挤占控制路径。
  status_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

void RobotRuntime::spinSome(std::chrono::milliseconds timeout) {
  // 寄生模式下没有自己的执行器：静默返回。
  // 抢别人的执行器是错的 —— 同一个节点被两个执行器 spin 是未定义行为。
  if (!executor_) {
    return;
  }
  executor_->spin_once(timeout);
}

void RobotRuntime::spin() {
  if (executor_) {
    executor_->spin();
  }
}

void RobotRuntime::stop() {
  if (executor_) {
    executor_->cancel();
  }
}

}  // namespace robot_sdk
