#include "robot_sdk/config.hpp"

#include <string>
#include <vector>

namespace robot_sdk {
namespace {

/// 声明参数（用默认值）并读回。
/// ROS 2 强制"先声明后使用"：这不是刁难，它让你把参数名拼错时立刻抛异常，
/// 而不是静默地永远拿到默认值。
template <typename T>
T declareAndGet(rclcpp::Node &node, const std::string &name,
                const T &default_value) {
  node.declare_parameter<T>(name, default_value);
  return node.get_parameter(name).get_value<T>();
}

/// 有些参数已经被 rclcpp::Node 预先声明过了（尤其 `use_sim_time`，
/// 每个节点都有），再声明一次会抛 ParameterAlreadyDeclaredException。
/// 所以这里先检查存在性。
template <typename T>
T getExistingOrDeclare(rclcpp::Node &node, const std::string &name,
                       const T &default_value) {
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
  return node.get_parameter(name).get_value<T>();
}

} // namespace

// SubscriptionQos：QoS 的全部知识都在这个类型里。
rclcpp::ReliabilityPolicy SubscriptionQos::reliabilityFrom(const std::string &value) {
  if (value == "best_effort") return rclcpp::ReliabilityPolicy::BestEffort;
  if (value == "reliable") return rclcpp::ReliabilityPolicy::Reliable;
  if (value == "system_default") return rclcpp::ReliabilityPolicy::SystemDefault;
  return rclcpp::ReliabilityPolicy::Unknown;
}

rclcpp::QoS SubscriptionQos::toQos() const {
  // depth <= 0 时退回 1，避免拼出深度 0 的非法 QoS。
  rclcpp::QoS qos(rclcpp::KeepLast(depth > 0 ? static_cast<std::size_t>(depth) : 1u));
  qos.reliability(reliabilityFrom(reliability));
  return qos;
}

SubscriptionQos SubscriptionQos::fromNode(rclcpp::Node &node, const std::string &prefix) {
  SubscriptionQos qos;
  qos.depth = declareAndGet<int>(node, prefix + ".depth", qos.depth);
  qos.reliability =
      declareAndGet<std::string>(node, prefix + ".reliability", qos.reliability);
  return qos;
}

RobotConfig RobotConfig::fromNode(rclcpp::Node &node) {
  RobotConfig cfg;

  // 用已有节点的名字，而不是默认的 "robot_sdk"。
  cfg.node_name = node.get_name();
  // ⚠️ 这个参数只在**自建模式**下真的有用，而那个模式下又**没有节点**可以声明参数 ——
  //    见 config.hpp 里 node_namespace 那一整段说明。之所以还留着，是为了让
  //    "先拿个临时节点读参数、再 Robot(cfg) 自建"这条路走得通。
  cfg.node_namespace =
      declareAndGet<std::string>(node, "node_namespace", cfg.node_namespace);
  // 注意这里用的是 getExistingOrDeclare —— 见上面的说明。
  cfg.use_sim_time =
      getExistingOrDeclare<bool>(node, "use_sim_time", cfg.use_sim_time);

  cfg.navigation.action_name = declareAndGet<std::string>(
      node, "navigation.action_name", cfg.navigation.action_name);
  cfg.navigation.global_frame = declareAndGet<std::string>(
      node, "navigation.global_frame", cfg.navigation.global_frame);
  cfg.navigation.base_frame = declareAndGet<std::string>(
      node, "navigation.base_frame", cfg.navigation.base_frame);
  cfg.navigation.default_timeout_s = declareAndGet<double>(
      node, "navigation.default_timeout_s", cfg.navigation.default_timeout_s);
  cfg.navigation.arrival_tolerance_m = declareAndGet<double>(
      node, "navigation.arrival_tolerance_m", cfg.navigation.arrival_tolerance_m);
  cfg.navigation.arrival_yaw_tolerance_rad =
      declareAndGet<double>(node, "navigation.arrival_yaw_tolerance_rad",
                            cfg.navigation.arrival_yaw_tolerance_rad);
  cfg.navigation.server_wait_timeout_s =
      declareAndGet<double>(node, "navigation.server_wait_timeout_s",
                            cfg.navigation.server_wait_timeout_s);

  cfg.motion.action_name = declareAndGet<std::string>(
      node, "motion.action_name", cfg.motion.action_name);
  cfg.motion.pose_action_name = declareAndGet<std::string>(
      node, "motion.pose_action_name", cfg.motion.pose_action_name);
  cfg.motion.pose_frame = declareAndGet<std::string>(
      node, "motion.pose_frame", cfg.motion.pose_frame);
  cfg.motion.joint_names =
      declareAndGet<std::vector<std::string>>(node, "motion.joint_names", cfg.motion.joint_names);
  cfg.motion.default_timeout_s = declareAndGet<double>(
      node, "motion.default_timeout_s", cfg.motion.default_timeout_s);
  cfg.motion.gripper_joint = declareAndGet<std::string>(
      node, "motion.gripper_joint", cfg.motion.gripper_joint);
  cfg.motion.gripper_open_position = declareAndGet<double>(
      node, "motion.gripper_open_position", cfg.motion.gripper_open_position);
  cfg.motion.gripper_closed_position = declareAndGet<double>(
      node, "motion.gripper_closed_position", cfg.motion.gripper_closed_position);
  cfg.motion.gripper_timeout_s = declareAndGet<double>(
      node, "motion.gripper_timeout_s", cfg.motion.gripper_timeout_s);
  // ⚠️ motion.named_targets 故意不在这里读：它是 map，而 ROS 2 参数只有
  // bool/int64/double/string 及其数组，没有 map 类型 ⇒ 只能由代码直接设置。

  cfg.perception.laser_scan_topic = declareAndGet<std::string>(
      node, "perception.laser_scan_topic", cfg.perception.laser_scan_topic);
  cfg.perception.imu_topic = declareAndGet<std::string>(
      node, "perception.imu_topic", cfg.perception.imu_topic);
  cfg.perception.depth_image_topic = declareAndGet<std::string>(
      node, "perception.depth_image_topic", cfg.perception.depth_image_topic);
  cfg.perception.point_cloud_topic = declareAndGet<std::string>(
      node, "perception.point_cloud_topic", cfg.perception.point_cloud_topic);
  cfg.perception.camera_info_topic = declareAndGet<std::string>(
      node, "perception.camera_info_topic", cfg.perception.camera_info_topic);
  cfg.perception.sensor_timeout_s = declareAndGet<double>(
      node, "perception.sensor_timeout_s", cfg.perception.sensor_timeout_s);

  cfg.status.status_topic = declareAndGet<std::string>(
      node, "status.status_topic", cfg.status.status_topic);
  cfg.status.joint_states_topic = declareAndGet<std::string>(
      node, "status.joint_states_topic", cfg.status.joint_states_topic);
  cfg.status.get_status_service = declareAndGet<std::string>(
      node, "status.get_status_service", cfg.status.get_status_service);
  cfg.status.default_timeout_s = declareAndGet<double>(
      node, "status.default_timeout_s", cfg.status.default_timeout_s);

  // 机器人侧属性：由执行侧（真机驱动 / 仿真器）使用，SDK 客户端不读。
  cfg.limits.goal_tolerance_m = declareAndGet<double>(
      node, "limits.goal_tolerance_m", cfg.limits.goal_tolerance_m);
  cfg.limits.yaw_tolerance_rad = declareAndGet<double>(
      node, "limits.yaw_tolerance_rad", cfg.limits.yaw_tolerance_rad);
  cfg.limits.max_speed = declareAndGet<double>(
      node, "limits.max_speed", cfg.limits.max_speed);
  cfg.limits.max_yaw_rate = declareAndGet<double>(
      node, "limits.max_yaw_rate", cfg.limits.max_yaw_rate);
  cfg.limits.joint_tolerance = declareAndGet<double>(
      node, "limits.joint_tolerance", cfg.limits.joint_tolerance);
  cfg.limits.default_joint_velocity = declareAndGet<double>(
      node, "limits.default_joint_velocity", cfg.limits.default_joint_velocity);
  cfg.limits.body_clearance_m = declareAndGet<double>(
      node, "limits.body_clearance_m", cfg.limits.body_clearance_m);

  // 传输层 QoS：集中在这里，参数名按角色展开（与结构体路径一致）。
  cfg.qos.sensor = SubscriptionQos::fromNode(node, "qos.sensor");
  cfg.qos.joint_states = SubscriptionQos::fromNode(node, "qos.joint_states");
  cfg.qos.status = SubscriptionQos::fromNode(node, "qos.status");

  return cfg;
}

}  // namespace robot_sdk
