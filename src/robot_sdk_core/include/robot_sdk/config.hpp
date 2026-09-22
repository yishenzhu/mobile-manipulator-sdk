#pragma once

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace robot_sdk {

/// 把命名空间规范化为 ROS 2 要求的规范形式：
///   - 非空则保证带前导 '/'   （"robot1"    → "/robot1"）
///   - 去掉全部尾部 '/'        （"/robot1/"  → "/robot1"）
///   - 只由 '/' 组成的输入归一成空串（表示根命名空间）
///
/// ROS 2 的规则是「namespace must not end with a '/', unless only a '/'」，
/// 违反它的输入会让 rclcpp::Node 构造直接抛 InvalidNamespaceException。
/// 配置由人手写，所以必须在这里兜住。
std::string normalizeNamespace(const std::string& ns);

/// 每个字段都有合理默认值，客户可以从 `RobotConfig{}` 开始，
/// 只覆盖本次部署真正不同的那几项。
/// 配置之后也可以从 ROS 参数读入（见 RobotConfig::fromNode），
/// 这样就能被 launch 文件覆盖而不用改代码。

struct NavigationConfig {
  std::string action_name{"navigate_to_pose"};
  std::string global_frame{"map"};
  // 用地面投影帧而不是 base_link：Pose2D 是平面量，语义上就该来自
  // "机器人在哪块地面上"（见 REP-105）。整个导航栈（Nav2 / SLAM）
  // 也用这个帧，保持全项目一致。
  std::string base_frame{"base_footprint"};
  double default_timeout_s{60.0};
  double goal_tolerance_m{0.05};
  double server_wait_timeout_s{5.0};
};

struct MotionConfig {
  std::string action_name{"move_to_joint_positions"};
  std::vector<std::string> joint_names{};
  double default_timeout_s{30.0};
  double default_max_velocity{0.5};
  std::map<std::string, std::vector<double>> named_targets{};
  std::string gripper_joint{"gripper_joint"};
  double gripper_open_position{0.04};
  double gripper_closed_position{0.0};
};

struct PerceptionConfig {
  std::string laser_scan_topic{"scan"};
  std::string imu_topic{"imu"};
  std::string depth_image_topic{"camera/depth/image_raw"};
  std::string point_cloud_topic{"camera/depth/points"};
  std::string camera_info_topic{"camera/depth/camera_info"};
  int queue_depth{10};
  double sensor_timeout_s{5.0};
};

struct StatusConfig {
  std::string status_topic{"robot_status"};
  std::string joint_states_topic{"joint_states"};
  std::string get_status_service{"get_robot_status"};
  double default_timeout_s{5.0};
};

struct RobotConfig {
  std::string node_name{"robot_sdk"};
  std::string robot_namespace{};
  bool use_sim_time{false};

  NavigationConfig navigation{};
  MotionConfig motion{};
  PerceptionConfig perception{};
  StatusConfig status{};

  /// 从 `node` 上声明的 ROS 参数填充配置。
  /// 参数名镜像结构体路径，例如 `navigation.global_frame`。
  /// 这样配置就能被 launch 文件覆盖，不用改代码重编。
  static RobotConfig fromNode(rclcpp::Node& node);

  /// 把相对话题/动作/服务名解析到 robot_namespace 之下。
  /// 绝对名（以 '/' 开头）原样返回。
  std::string resolve(const std::string& relative_name) const;
};

}  // namespace robot_sdk
