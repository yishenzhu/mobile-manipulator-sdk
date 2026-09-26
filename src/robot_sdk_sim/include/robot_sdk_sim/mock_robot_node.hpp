#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <robot_sdk_interfaces/action/move_to_joint_positions.hpp>
#include <robot_sdk_interfaces/action/navigate_to_pose.hpp>
#include <robot_sdk_interfaces/msg/robot_status.hpp>
#include <robot_sdk_interfaces/srv/get_robot_status.hpp>

#include "robot_sdk/config.hpp"
#include "robot_sdk/detail/action_slot.hpp"
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

  /// 按最后收到的 `cmd_vel` 推进位姿 —— 跑在 50 Hz 控制回路上。
  /// 有了它，Nav2 这类"出口是 cmd_vel"的导航栈就能直接驱动本模拟器。
  void cmdVelTick();

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

  /// 组装一条运动反馈。参数 `lock` 是凭证（同 `makeStatus`）。
  MoveToJointPositions::Feedback makeMotionFeedback(std::lock_guard<std::mutex>& lock) const;

  void publishFrames();
  void publishJoints();

  /// 发布板载传感器（激光 + IMU + 相机），内部降到 10 Hz。
  /// 它只负责**节奏**和**发布**；每一路的数据在下面的 `makeXxx` 里组装。
  /// 与状态、关节一样：**锁内取数、锁外发布**（本文档 §4.3 纪律 1）。
  void publishSensors();

  /// 组装各路传感器数据 —— 与 `makeStatus` / `makeNavFeedback` 同一模式：
  /// 在**锁内**被调用，`lock` 是**凭证**（参数不命名 = 一眼看出它不是数据）。
  ///
  /// `previous_yaw` 由调用方传进来（它读的是同一个 `pose_`），
  /// 这样 `makeImu` 仍然是 `const` —— 它不必去改 `last_yaw_`。
  sensor_msgs::msg::LaserScan makeScan(std::lock_guard<std::mutex>& lock,
                                       const rclcpp::Time& stamp) const;
  sensor_msgs::msg::Imu makeImu(std::lock_guard<std::mutex>& lock,
                                const rclcpp::Time& stamp, double previous_yaw) const;
  /// 内参是**常量**，不读任何共享状态 —— 所以它不需要凭证，也不收 lock。
  sensor_msgs::msg::CameraInfo makeCameraInfo(const rclcpp::Time& stamp) const;
  /// 深度图要逐像素射线求交，**读 `pose_`** —— 需要凭证。
  sensor_msgs::msg::Image makeDepthImage(std::lock_guard<std::mutex>& lock,
                                         const rclcpp::Time& stamp) const;
  /// 点云**从深度图反投影**得到（信息量相同，只是排布从"表格"变成"散点"）。
  /// 它只吃已经算好的深度图、不读任何共享状态 —— 所以不需要凭证，也不收 lock。
  sensor_msgs::msg::PointCloud2 makePointCloud(const sensor_msgs::msg::Image& depth) const;

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
  /// 状态查询服务的组。**独立于 `timer_group_`**：查询是客户触发的，
  /// 不该跟 50 Hz 控制回路共用一条车道（实测见 学习笔记.md §7.9）。
  rclcpp::CallbackGroup::SharedPtr service_group_;
  /// 导航动作服务端的组。**故意与 `timer_group_` 相反（可重入）**：
  /// 目标/取消可以在控制回路跑着的时候到达 —— 两条车道真的并发。
  rclcpp::CallbackGroup::SharedPtr action_group_;
  /// 底盘命令的组，独立于其它车道。话题名由 mock 自己的参数 `cmd_vel_topic` 决定
  /// （它不属于 SDK 契约 —— 客户从不发 cmd_vel）。
  rclcpp::CallbackGroup::SharedPtr cmd_group_;

  rclcpp::Publisher<RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::Service<GetRobotStatus>::SharedPtr status_service_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr nav_server_;
  rclcpp_action::Server<MoveToJointPositions>::SharedPtr motion_server_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// 保护下面这些"机器人状态"成员：控制定时器写、状态/反馈读。
  std::mutex mutex_;
  Pose2D pose_{};

  // ── 导航状态（同样受 mutex_ 保护）──
  /// 动作状态收在**一个槽位**里（三态 + 句柄 + 一位取消信箱）。
  /// 以前这里是 5 个散装成员、其中 3 个 bool —— 见 action_slot.hpp 的说明。
  ActionSlot<NavigateToPose> nav_;
  Pose2D nav_target_{};
  double nav_max_speed_{0.0};
  /// 反馈降频用：每 N 个控制周期发一条反馈。
  int nav_feedback_divider_{0};

  // ── 关节运动状态（同样受 mutex_ 保护）──
  ActionSlot<MoveToJointPositions> motion_;
  /// 目标位置，与 `joint_positions_` **同序同长**；目标没点名的关节填当前值。
  std::vector<double> motion_target_;
  double motion_max_velocity_{0.0};
  /// 起始最大误差 —— `completion_percentage` 的分母。
  double motion_initial_error_{0.0};
  /// 反馈降频用：每 N 个控制周期发一条反馈。
  int motion_feedback_divider_{0};
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  double battery_{100.0};
  bool charging_{false};
  double uptime_s_{0.0};

  // ── 底盘命令（同样受 mutex_ 保护）──
  /// 最近一次 cmd_vel 的线速度 / 角速度，以及它是什么时候收到的。
  /// 收不到新命令超过一段时间就当停车 —— 真底盘也是这个行为。
  /// 时间用**单调时钟**：只问"距上次过了多久"，与系统时间/仿真时间无关。
  double cmd_linear_{0.0};
  double cmd_angular_{0.0};
  bool cmd_received_{false};
  std::chrono::steady_clock::time_point cmd_stamp_;

  /// 传感器降频用：控制回路 50 Hz，传感器只要 10 Hz。
  /// 它和 `last_yaw_` 其实只有控制回路这一条车道会碰（严格说不需要锁），
  /// 但**故意**留在 `mutex_` 保护区内 —— 理由写在 `publishSensors()` 里。
  int sensor_divider_{0};
  /// 上一拍的偏航角 —— IMU 的角速度只能由差分得到，没有别的来源。
  double last_yaw_{0.0};

  // 传感器挂载点（相对 base_frame 的静态变换里用得到）
  std::string laser_frame_{"laser_frame"};
  std::string imu_frame_{"imu_link"};
  std::string camera_frame_{"camera_link"};
  /// 相机的**光学帧**。ROS 约定图像数据的坐标系是：z 朝前、x 朝右、y 朝下；
  /// 而 body 系是 x 前 / y 左 / z 上 —— 两者差一个固定的旋转。
  /// 少广播这一帧，客户拿 TF 去投影深度图会得到**错误但看起来合理**的结果。
  std::string camera_optical_frame_{"camera_optical_frame"};
};

}  // namespace robot_sdk::sim
