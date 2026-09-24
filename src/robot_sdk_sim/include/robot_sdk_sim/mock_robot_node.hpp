#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <robot_sdk_interfaces/action/move_to_joint_positions.hpp>
#include <robot_sdk_interfaces/action/navigate_to_pose.hpp>
#include <robot_sdk_interfaces/msg/robot_status.hpp>
#include <robot_sdk_interfaces/srv/get_robot_status.hpp>

#include "robot_sdk/config.hpp"
#include "robot_sdk/types.hpp"

namespace robot_sdk::sim {

/// 自包含的模拟移动操作机器人。
///
/// 它只讲 **SDK 的契约**：实现 SDK 的动作服务端、发布 SDK 会订阅的
/// 话题、广播 TF。因此示例、集成测试和新手上手都**不需要 Gazebo 或真机**。
///
/// 命名空间由**节点自身**决定（见 launch 文件），而不是某个参数 ——
/// 话题名一律用相对名，交给 rcl 按节点命名空间展开。
class MockRobotNode : public rclcpp::Node {
 public:
  explicit MockRobotNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  using GetRobotStatus = robot_sdk_interfaces::srv::GetRobotStatus;
  using RobotStatus = robot_sdk_interfaces::msg::RobotStatus;
  using NavigateToPose = robot_sdk_interfaces::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using MoveToJointPositions = robot_sdk_interfaces::action::MoveToJointPositions;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveToJointPositions>;

  void controlTick();
  void statusTick();

  /// 推进导航 —— 跑在 50 Hz 控制回路上。
  /// 锁内只碰数据；`publish_feedback` / `succeed` / `canceled` 全在锁外调用（笔记 §7.8）。
  void navTick();

  /// 导航动作服务端的三个回调。它们跑在**独立的可重入组**里，
  /// 与 50 Hz 控制回路真的并发 —— 这正是 `mutex_` 存在的理由。
  rclcpp_action::GoalResponse handleNavGoal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse handleNavCancel(
      const std::shared_ptr<GoalHandleNavigate> goal_handle);
  void handleNavAccepted(const std::shared_ptr<GoalHandleNavigate> goal_handle);

  /// 组装一条导航反馈。参数 `lock` 是凭证（同 `makeStatus`）。
  NavigateToPose::Feedback makeNavFeedback(std::lock_guard<std::mutex>& lock) const;

  /// 推进关节运动 —— 同样跑在 50 Hz 控制回路上，结构同 `navTick`（笔记 §7.8）。
  void motionTick();

  /// 关节运动服务端的三个回调。与导航**共用** `action_group_`（同一条可重入车道）。
  rclcpp_action::GoalResponse handleMotionGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveToJointPositions::Goal> goal);
  rclcpp_action::CancelResponse handleMotionCancel(
      const std::shared_ptr<GoalHandleMove> goal_handle);
  void handleMotionAccepted(const std::shared_ptr<GoalHandleMove> goal_handle);

  void publishFrames();
  void publishJoints();

  /// 把"此刻的状态"拼成一条 RobotStatus。
  ///
  /// **状态话题与服务查询共用它** —— 否则同一个事实存两份，
  /// 迟早出现"话题说 A、服务说 B"这种极难查的不一致
  /// （本类一开始就踩过：服务那份漏了 error_code 与 is_estop_engaged）。
  ///
  /// 参数 `lock` 是**凭证**：想调用它，必须先持有 `mutex_`。
  ///
  /// **为什么用凭证，而不是让它自己加锁**：自己加锁会让"在已持锁的地方顺手调用"
  /// 变成**自死锁**（`std::mutex` 检测不到自锁，是 UB 不是异常，直接挂住）——
  /// 而那种误用太容易发生。伪造一把锁则必须**刻意**构造别的 mutex 的 `lock_guard`，
  /// 现实中不会发生。判据是**触发难度**，不是理论严密性。详见笔记 §7.7。
  RobotStatus makeStatus(std::lock_guard<std::mutex>& lock) const;

  /// 按名字找一个关节；没有就新建一个，返回其下标。
  std::size_t ensureJoint(const std::string& name);

  RobotConfig config_;

  rclcpp::CallbackGroup::SharedPtr timer_group_;
  /// 导航动作服务端的组。**故意与 `timer_group_` 相反（可重入）**：
  /// 目标/取消可以在控制回路跑着的时候到达 —— 两条车道真的并发。
  rclcpp::CallbackGroup::SharedPtr action_group_;

  rclcpp::Publisher<RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::Service<GetRobotStatus>::SharedPtr status_service_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr nav_server_;
  rclcpp_action::Server<MoveToJointPositions>::SharedPtr motion_server_;

  /// 保护下面这些"机器人状态"成员：控制定时器写、状态/反馈读。
  std::mutex mutex_;
  Pose2D pose_{};

  // ── 导航状态（同样受 mutex_ 保护）──
  std::shared_ptr<GoalHandleNavigate> nav_goal_handle_;
  Pose2D nav_target_{};
  double nav_max_speed_{0.0};
  bool nav_active_{false};
  /// 客户已经请求取消。由取消回调（框架线程）置位，控制回路消费。
  bool nav_cancel_requested_{false};
  /// 反馈降频用：每 N 个控制周期发一条反馈。
  int feedback_divider_{0};

  // ── 关节运动状态（同样受 mutex_ 保护）──
  std::shared_ptr<GoalHandleMove> motion_goal_handle_;
  /// 目标位置，与 `joint_positions_` **同序同长**；目标没点名的关节填当前值。
  std::vector<double> motion_target_;
  double motion_max_velocity_{0.0};
  bool motion_active_{false};
  bool motion_cancel_requested_{false};
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  double battery_{100.0};
  bool charging_{false};
  double uptime_s_{0.0};

  // 传感器挂载点（相对 base_frame 的静态变换里用得到）
  std::string laser_frame_{"laser_frame"};
  std::string imu_frame_{"imu_link"};
  std::string camera_frame_{"camera_link"};
};

}  // namespace robot_sdk::sim
