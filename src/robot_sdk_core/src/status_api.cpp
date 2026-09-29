#include "robot_sdk/status_api.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/qos.hpp>

namespace robot_sdk {

StatusAPI::StatusAPI(const rclcpp::Node::SharedPtr& node,
                     const rclcpp::CallbackGroup::SharedPtr& callback_group,
                     const StatusConfig& config)
    : node_(node), config_(config), callback_group_(callback_group) {
  rclcpp::SubscriptionOptions options;
  // ★ 与 PerceptionAPI 同理：回调组决定线程语义。
  //   忘了传 → 落进节点的默认组（互斥），与客户自己的回调互相拖累。
  options.callback_group = callback_group_;

  // 状态是低频、且"最新一条才有意义"，所以深度 1 + reliable。
  rclcpp::QoS status_qos(rclcpp::KeepLast(1));
  status_qos.reliable();

  // 话题名用相对名，由 rcl 按节点命名空间展开（见 学习笔记.md 第 15 节）。
  status_subscription_ = node_->create_subscription<RobotStatus>(
      config_.status_topic, status_qos,
      [this](RobotStatus::ConstSharedPtr msg) {
        status_cache_.update(msg);
        // 先**在锁内拷一份回调列表**，再**在锁外**调用 ——
        // 绝不在持锁状态下调用用户回调（它可能很慢，甚至反过来调本对象的方法）。
        std::vector<StatusCallback> listeners;
        {
          std::lock_guard<std::mutex> lock(listeners_mutex_);
          listeners = listeners_;
        }
        for (const auto& listener : listeners) {
          listener(*msg);
        }
      },
      options);

  joint_state_subscription_ = node_->create_subscription<JointState>(
      config_.joint_states_topic, rclcpp::SensorDataQoS(),
      [this](JointState::ConstSharedPtr msg) { joint_state_cache_.update(msg); }, options);

  // service client 也要指定回调组，否则响应回调会落到默认组。
  // QoS 用 rclcpp::ServicesQoS() —— 与旧写法等价，而 rmw_qos_profile_t 重载在 Jazzy 已 deprecated。
  status_client_ = node_->create_client<GetRobotStatus>(
      config_.get_status_service, rclcpp::ServicesQoS(), callback_group_);
}

std::optional<StatusAPI::RobotStatusPtr> StatusAPI::status() const {
  return status_cache_.latest();
}

std::optional<StatusAPI::JointStatePtr> StatusAPI::jointStates() const {
  return joint_state_cache_.latest();
}

Result<StatusAPI::RobotStatusPtr> StatusAPI::waitForStatus(
    std::chrono::duration<double> timeout) {
  auto value = status_cache_.waitForValue(timeout);
  if (!value) {
    return Error::Timeout("no robot status received within timeout");
  }
  return *value;
}

Result<StatusAPI::JointStatePtr> StatusAPI::waitForJointStates(
    std::chrono::duration<double> timeout) {
  auto value = joint_state_cache_.waitForValue(timeout);
  if (!value) {
    return Error::Timeout("no joint states received within timeout");
  }
  return *value;
}

Result<StatusAPI::RobotStatus> StatusAPI::queryStatus(std::chrono::duration<double> timeout) {
  if (!status_client_->wait_for_service(timeout)) {
    return Error::Unavailable("status service '" + config_.get_status_service +
                              "' is not available");
  }

  auto request = std::make_shared<GetRobotStatus::Request>();
  request->include_joint_states = true;

  auto future = status_client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    return Error::Timeout("status service did not respond in time");
  }
  auto response = future.get();
  if (!response) {
    return Error::Internal("status service returned a null response");
  }
  // 又是那两层错误语义：通信成功 ≠ 服务报告成功。
  if (!response->success) {
    return Error::Unavailable("status service reported failure: " + response->message);
  }
  // 若流式的 joint_states 还没来过，就用服务返回的这份先填上。
  if (!joint_state_cache_.hasValue() && !response->joint_states.name.empty()) {
    joint_state_cache_.update(std::make_shared<const JointState>(response->joint_states));
  }
  return response->status;
}

rclcpp::Subscription<StatusAPI::RobotStatus>::SharedPtr StatusAPI::onStatus(
    StatusCallback callback) {
  std::lock_guard<std::mutex> lock(listeners_mutex_);
  listeners_.push_back(std::move(callback));
  return status_subscription_;
}

bool StatusAPI::isHealthy() const {
  const auto current = status_cache_.latest();
  if (!current || !*current) {
    // 没有数据 ≠ 健康。宁可说"不知道"，也不给一个可能错的 true。
    return false;
  }
  return (*current)->error_code == 0 &&
         (*current)->state != RobotStatus::STATE_ERROR &&
         !(*current)->is_estop_engaged;
}

float StatusAPI::batteryPercentage() const {
  const auto current = status_cache_.latest();
  if (!current || !*current) {
    return std::nanf("");
  }
  return (*current)->battery_percentage;
}

}  // namespace robot_sdk
