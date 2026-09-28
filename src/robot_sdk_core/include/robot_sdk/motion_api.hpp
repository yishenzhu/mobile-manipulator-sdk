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
#include <robot_sdk_interfaces/action/move_to_pose.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "robot_sdk/action_handle.hpp"
#include "robot_sdk/config.hpp"
#include "robot_sdk/result.hpp"

namespace robot_sdk {

/// 关节空间运动目标。
struct MotionGoal {
  std::vector<std::string> joint_names{};
  std::vector<double> positions{};

  /// 速度上限。**<= 0 = 不指定** —— 由机器人按自己的上限决定（那是它的属性，
  /// 写在它的 URDF 里；SDK 不该替它记一份）。
  double max_velocity{0.0};

  /// 加速度上限。**<= 0 = 由后端决定。**
  /// 兑现情况：`motion_adapter` 把它折算进轨迹时长；mock 走梯形速度曲线。两侧都读。
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

/// 笛卡尔空间目标：把末端执行器挪到某个位姿。
/// 逆解与规划是**执行侧**的事（那边就是 MoveIt），这里只表达"我想让末端到哪儿"。
struct PoseGoal {
  /// 坐标系由 `header.frame_id` 说 —— 默认 `map`：客户想的是"世界上那一点"，
  /// 用 `base_link` 的话底盘一挪目标就跟着跑。
  geometry_msgs::msg::PoseStamped target_pose{};

  /// true = 朝向随你（规划器自己在可行解里挑）；false = 照 `target_pose` 里的姿态。
  /// 想"保持当前姿态"不用这个开关：从 `TransformManager::lookupTransform()` 读现在的填进来即可。
  bool any_orientation{false};

  /// 速度上限。**<= 0 = 不指定**（同 `MotionGoal`）。
  double max_velocity{0.0};

  /// 超时秒数。**<= 0 = 用 `MotionConfig::default_timeout_s`。**
  double timeout_s{0.0};
};

struct PoseResult {
  bool success{false};
  double final_position_error{0.0};
  /// 姿态误差（弧度）。`any_orientation` 为真时**恒为 0** —— 不是"没测"，
  /// 是本来就没要求（区别写进了 action 的字段注释）。
  double final_orientation_error{0.0};
  std::string message{};
};

struct PoseFeedback {
  geometry_msgs::msg::PoseStamped current_pose{};
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
  using PoseAction = robot_sdk_interfaces::action::MoveToPose;
  using PoseFeedbackCallback = std::function<void(const PoseFeedback&)>;

  MotionAPI(const rclcpp::Node::SharedPtr& node,
            const rclcpp::CallbackGroup::SharedPtr& callback_group, const MotionConfig& config);

  bool serverAvailable(std::chrono::duration<double> timeout);

  std::shared_ptr<ActionHandle<Action>> moveToJointPositionsAsync(
      const MotionGoal& goal, FeedbackCallback on_feedback = nullptr);

  Result<MotionResult> moveToJointPositions(const MotionGoal& goal);

  /// 笛卡尔空间运动：把末端执行器挪到 `goal.target_pose`。
  /// 走的是 `MotionConfig::pose_action_name` 那条后端 —— 和关节运动是**两个** action。
  std::shared_ptr<ActionHandle<PoseAction>> moveToPoseAsync(
      const PoseGoal& goal, PoseFeedbackCallback on_feedback = nullptr);

  Result<PoseResult> moveToPose(const PoseGoal& goal);

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
  std::shared_ptr<rclcpp_action::Client<PoseAction>> pose_client_;

  mutable std::mutex handles_mutex_;
  std::vector<std::weak_ptr<ActionHandle<Action>>> active_handles_;
  std::vector<std::weak_ptr<ActionHandle<PoseAction>>> active_pose_handles_;
};

}  // namespace robot_sdk
