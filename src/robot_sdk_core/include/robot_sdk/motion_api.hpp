#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_sdk_interfaces/action/move_to_joint_positions.hpp>

#include "robot_sdk/action_handle.hpp"
#include "robot_sdk/config.hpp"
#include "robot_sdk/result.hpp"

namespace robot_sdk {

/// 关节空间运动目标。
struct MotionGoal {
  std::vector<std::string> joint_names{};
  std::vector<double> positions{};

  /// 速度上限。**<= 0 = 用 `MotionConfig::default_max_velocity`。**
  double max_velocity{0.0};

  /// 加速度上限。**<= 0 = 由后端决定。**
  /// ⚠️ 当前随 SDK 提供的后端（mock / 适配器）**都还没读它** ——
  /// 见 学习笔记.md 的「声明了但没兑现」清单。
  double max_acceleration{0.0};

  /// 超时秒数。**<= 0 = 用 `MotionConfig::default_timeout_s`。**
  double timeout_s{0.0};
};

/// 运动的终态。与 `Result` 是**两层**语义：
/// `Result.ok()` 是"流程走完了"，`success` 才是"关节真的到位了"。
struct MotionResult {
  bool success{false};
  double final_position_error{0.0};
  std::string message{};
};

struct MotionFeedback {
  std::vector<std::string> joint_names{};
  std::vector<double> positions{};
  double completion_percentage{0.0};
};

/// 关节空间运动，含机械臂与夹爪。
///
/// 对准的是 **SDK 自己的运动 action**；实际部署时由一层适配器转到
/// MoveIt / ros2_control 上，而客户感觉不到。
class MotionAPI {
 public:
  using Action = robot_sdk_interfaces::action::MoveToJointPositions;
  using FeedbackCallback = std::function<void(const MotionFeedback&)>;

  MotionAPI(const rclcpp::Node::SharedPtr& node,
            const rclcpp::CallbackGroup::SharedPtr& callback_group, const MotionConfig& config);

  bool serverAvailable(std::chrono::duration<double> timeout);

  std::shared_ptr<ActionHandle<Action>> moveToJointPositionsAsync(
      const MotionGoal& goal, FeedbackCallback on_feedback = nullptr);

  Result<MotionResult> moveToJointPositions(const MotionGoal& goal);

  /// 移动到 `MotionConfig::named_targets` 里声明的具名位姿（如 "home"）。
  ///
  /// ⚠️ 那个字段是 `map<string, vector<double>>`，而 **ROS 2 参数没有 map 类型**，
  ///    所以它**无法通过 launch 文件配置**，只能由代码赋值（见 学习笔记.md 的 C4）。
  Result<MotionResult> moveToNamedTarget(const std::string& name);

  Result<MotionResult> home();

  Result<MotionResult> setGripper(double position);
  Result<MotionResult> openGripper();
  Result<MotionResult> closeGripper();

  /// 取消本 API 派发过的所有目标；顺手清理已过期的登记。
  void cancelAll();

 private:
  Error validate(const MotionGoal& goal) const;

  rclcpp::Node::SharedPtr node_;
  MotionConfig config_;
  std::shared_ptr<rclcpp_action::Client<Action>> client_;

  mutable std::mutex handles_mutex_;
  std::vector<std::weak_ptr<ActionHandle<Action>>> active_handles_;
};

}  // namespace robot_sdk
