#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <robot_sdk_interfaces/msg/robot_status.hpp>
#include <robot_sdk_interfaces/srv/get_robot_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robot_sdk/config.hpp"
#include "robot_sdk/detail/latest_value_cache.hpp"
#include "robot_sdk/result.hpp"

namespace robot_sdk {

/// 机器人的健康、电源与故障上报。
///
/// 状态**既以流的形式推送、也能按需查询**，所以这里两种都提供：
/// 一份缓存的快照，以及一次请求/响应的调用。
class StatusAPI {
 public:
  using RobotStatus = robot_sdk_interfaces::msg::RobotStatus;
  using JointState = sensor_msgs::msg::JointState;
  using GetRobotStatus = robot_sdk_interfaces::srv::GetRobotStatus;
  using StatusCallback = std::function<void(const RobotStatus&)>;

  // 快照一律用 ConstSharedPtr（与 PerceptionAPI 同一规则）：
  // 消息本来就是中间件在堆上分配的，存指针是零额外成本，也避免了拷贝。
  using RobotStatusPtr = RobotStatus::ConstSharedPtr;
  using JointStatePtr = JointState::ConstSharedPtr;

  /// 只接收自己需要的那一份配置（接口隔离）。
  StatusAPI(const rclcpp::Node::SharedPtr& node,
            const rclcpp::CallbackGroup::SharedPtr& callback_group, const StatusConfig& config);

  /// 最近一次推送的状态；还没收到过则为 nullopt。
  std::optional<RobotStatusPtr> status() const;

  std::optional<JointStatePtr> jointStates() const;

  bool hasStatus() const { return status_cache_.hasValue(); }

  /// 阻塞等待一次状态；拿不到时返回带原因的 Error（与 PerceptionAPI 的等待同一形状）。
  Result<RobotStatusPtr> waitForStatus(std::chrono::duration<double> timeout);

  /// 通过 SDK 的服务接口做一次请求/响应式的状态查询。
  ///
  /// 这里返回**按值**的 RobotStatus（而快照返回 ConstSharedPtr）——
  /// 因为服务给的是一份**新值**、不是缓存快照，按值返回没有可省的拷贝；
  /// 硬套指针反而要额外分配一次，更慢又更绕。
  Result<RobotStatus> queryStatus(std::chrono::duration<double> timeout);

  /// 注册一个"每次状态更新都调用"的回调。
  /// 返回的订阅由本 API 持有；拿它主要是为了查 QoS，随手丢掉也没问题。
  rclcpp::Subscription<RobotStatus>::SharedPtr onStatus(StatusCallback callback);

  /// error_code == 0、不在 ERROR 状态、且没有急停。
  bool isHealthy() const;

  /// 电量百分比；状态未知时返回 NaN。
  float batteryPercentage() const;

 private:
  rclcpp::Node::SharedPtr node_;
  StatusConfig config_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  LatestValueCache<RobotStatusPtr> status_cache_;
  LatestValueCache<JointStatePtr> joint_state_cache_;

  rclcpp::Subscription<RobotStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Client<GetRobotStatus>::SharedPtr status_client_;

  std::mutex listeners_mutex_;
  std::vector<StatusCallback> listeners_;
};

}  // namespace robot_sdk
