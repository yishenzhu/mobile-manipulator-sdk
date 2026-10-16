#pragma once

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace robot_sdk {

/// 每个字段都有合理默认值，客户可以从 `RobotConfig{}` 开始，只覆盖本次部署不同的那几项。
/// 也能从 ROS 参数读入（见 RobotConfig::fromNode），从而被 launch 覆盖而不用改代码。

struct NavigationConfig {
  /// 后端 action 名（相对名，由 rcl 按节点命名空间展开）。
  /// ⚠️ 必须与 `MotionConfig::action_name` **不同** —— 两者是不同类型的 action，
  ///    同名会让底层 send_goal 服务话题类型撞车，rcl 拒绝创建。见 学习笔记.md 第 1 节。
  std::string action_name{"navigate_to_pose"};
  std::string global_frame{"map"};
  // 用地面投影帧而不是 base_link：Pose2D 是平面量，语义上就该来自
  // "机器人在哪块地面上"（见 REP-105）。整个导航栈（Nav2 / SLAM）
  // 也用这个帧，保持全项目一致。
  std::string base_frame{"base_footprint"};
  double default_timeout_s{60.0};
  double server_wait_timeout_s{5.0};
  /// 客户端**验收**判据：goTo() 拿到 result 后读回位姿自己核对，超差即把 success 判 false。
  /// ⚠️ 必须**比后端自己的到位容差更宽**（Nav2 默认 xy 0.15 / yaw 0.25）—— 验收是兜底，
  ///    只抓"离得很远却报成功"，比后端更严就会把正常结果误判成失败。
  double arrival_tolerance_m{0.25};
  double arrival_yaw_tolerance_rad{0.35};
};

struct MotionConfig {
  /// 后端 action 名。⚠️ 必须与 `NavigationConfig::action_name` **不同**（同上）。
  std::string action_name{"move_to_joint_positions"};
  /// 笛卡尔目标的后端 action 名。和上面是**两个不同类型的 action**，
  /// 所以名字也必须不同（理由同上：同名会让 send_goal 服务话题类型撞车）。
  std::string pose_action_name{"move_to_pose"};
  /// 笛卡尔目标的默认坐标系：客户没给 `header.frame_id` 时用它。
  /// 选 `map` 是因为客户想的是"世界上那一点" —— 用 base_link 的话底盘一挪目标就跟着跑。
  std::string pose_frame{"map"};
  std::vector<std::string> joint_names{};
  double default_timeout_s{30.0};
  /// 等"服务端接受目标"的上限，同 NavigationConfig::server_wait_timeout_s：
  /// 派发时等到接受才返回 handle，取消才不会有"还没有 goal handle"的空窗。
  double server_wait_timeout_s{5.0};
  std::map<std::string, std::vector<double>> named_targets{};
  std::string gripper_joint{"gripper_joint"};
  /// ⚠️ 默认值必须落在**参考机器人夹爪的行程内**（URDF 的 `gripper_joint` 是 [0, 0.03]）。
  /// 超出会被执行侧当场拒收（实测 0.04 拒、0.03 收）—— 所以别凭"4 cm 才像张开"去调它。
  double gripper_open_position{0.03};
  double gripper_closed_position{0.0};
  /// 夹爪动作的超时。夹爪行程短、动作快，所以**独立于 default_timeout_s** ——
  /// 用通用的 30 秒去等一个夹爪动作，卡住时要干等半分钟。
  double gripper_timeout_s{5.0};
  // ⚠️ 这里**没有**"连续关节 / 速度上限 / 关节限位" —— 它们是**机器人的属性**，写在 URDF 里、
  //    由执行侧读出来用。SDK 只发意图（去哪、多快），不替机器人判断"能不能"。
  //    别加回来：两处各存一份迟早漂移。
};

struct PerceptionConfig {
  std::string laser_scan_topic{"scan"};
  std::string imu_topic{"imu"};
  std::string depth_image_topic{"camera/depth/image_raw"};
  std::string point_cloud_topic{"camera/depth/points"};
  std::string camera_info_topic{"camera/depth/camera_info"};
  double sensor_timeout_s{5.0};
};

struct StatusConfig {
  std::string status_topic{"robot_status"};
  std::string joint_states_topic{"joint_states"};
  std::string get_status_service{"get_robot_status"};
  double default_timeout_s{5.0};
};

/// 一个订阅的传输参数（QoS）。业务配置管"订哪个话题"，这里管"怎么订"。
struct SubscriptionQos {
  /// KEEP_LAST 的队列深度（QoS 的 history depth）。
  int depth{10};
  /// 取值：best_effort | reliable | system_default；其余一律按 Unknown 处理。
  std::string reliability{"best_effort"};

  /// 拼出本订阅的 rclcpp QoS。
  rclcpp::QoS toQos() const;

  /// 字符串 → ReliabilityPolicy。只认上面三个值，其余（含 unknown / best_available）落成 Unknown，不抛。
  static rclcpp::ReliabilityPolicy reliabilityFrom(const std::string& value);

  /// 从 ROS 参数读（`<prefix>.depth` 与 `<prefix>.reliability`），默认值用本结构字段自身的。
  static SubscriptionQos fromNode(rclcpp::Node& node, const std::string& prefix);
};

/// 全局 QoS：一处配好、各模块按需取用（Robot 装配时按角色分发）。
/// 角色按数据性质分：sensor = 5 个感知话题，joint_states = /joint_states，status = /robot_status。
/// ⚠️ 默认值由兼容规则决定（订阅端只能比发布端更宽松）；推导与实测见 docs/qos.md。
struct QosConfig {
  SubscriptionQos sensor{10, "best_effort"};
  SubscriptionQos joint_states{5, "best_effort"};
  SubscriptionQos status{1, "reliable"};
};

/// **机器人自己的属性**（不是 SDK 的使用参数）：到位判据与运动能力上限。
/// 由**执行侧**（真机驱动 / 仿真器）读出来用；SDK 客户端只发意图，不替机器人判断"到了没有"。
/// ⚠️ 别把这些搬回 NavigationConfig：同一个概念两处各存一份，迟早漂移（同 MotionConfig 的说明）。
struct RobotLimitsConfig {
  /// 位置到位判据（服务端判断"我到了"的阈值）。
  double goal_tolerance_m{0.05};
  /// 朝向到位判据。
  double yaw_tolerance_rad{0.05};
  /// 底盘线/角速度**能力上限**（成对）。与 `NavigationGoal::max_speed`（本次请求）是两层：
  /// 请求归调用方，上限归机器人，执行侧取 min。
  double max_speed{0.5};
  double max_yaw_rate{1.5};
  /// 关节到位容差（弧度 / 米），执行侧判"到位"用。
  double joint_tolerance{0.01};
  /// 客户没给 `max_velocity` 时的**默认**关节速度（不是上限 —— 上限在 URDF 的关节 limit 里）。
  double default_joint_velocity{0.5};
  /// 避障预检的车身余量：朝目标看出去，离墙比"剩余距离"还近这么多就不往前开。
  double body_clearance_m{0.3};
};

struct RobotConfig {
  /// 自建节点时用的**节点名**。
  std::string node_name{"robot_sdk"};
  /// 自建节点时用的**命名空间**（和 node_name 配对，两者正交，见 runtime.cpp）。
  /// 它只影响节点的命名空间，不参与拼话题名 —— 相对名由 rcl 展开。
  /// ⚠️ 和 node_name 一样只在自建模式下有用；寄生模式下被忽略。
  std::string node_namespace{};
  bool use_sim_time{false};

  NavigationConfig navigation{};
  MotionConfig motion{};
  PerceptionConfig perception{};
  StatusConfig status{};
  /// 机器人侧属性（服务端用；SDK 客户端不读）。
  RobotLimitsConfig limits{};
  /// 传输层的 QoS 集中在这一处；各 API 由 Robot 装配时按角色取用。
  QosConfig qos{};

  /// 从 `node` 上声明的 ROS 参数填充配置。
  /// 参数名镜像结构体路径，例如 `navigation.global_frame`。
  /// 这样配置就能被 launch 文件覆盖，不用改代码重编。
  static RobotConfig fromNode(rclcpp::Node& node);
};

}  // namespace robot_sdk
