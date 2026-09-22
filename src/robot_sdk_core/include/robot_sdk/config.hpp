#pragma once

#include <map>
#include <string>
#include <vector>

namespace robot_sdk {

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
};

}  // namespace robot_sdk
