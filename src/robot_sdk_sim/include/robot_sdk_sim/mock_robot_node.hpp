#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

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

  void controlTick();
  void statusTick();

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

  rclcpp::Publisher<RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::Service<GetRobotStatus>::SharedPtr status_service_;

  /// 保护下面这些"机器人状态"成员：控制定时器写、状态/反馈读。
  std::mutex mutex_;
  Pose2D pose_{};
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
