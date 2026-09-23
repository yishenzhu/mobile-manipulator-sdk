#include "robot_sdk/config.hpp"

#include <string>
#include <vector>

namespace robot_sdk {
namespace {

/// 声明参数（用默认值）并读回。
///
/// 为什么必须先声明再读：ROS 2 强制参数先声明后使用。
/// 这不是刁难 —— 它让你把参数名拼错时立刻抛异常，
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

RobotConfig RobotConfig::fromNode(rclcpp::Node &node) {
  RobotConfig cfg;

  // 用已有节点的名字，而不是默认的 "robot_sdk"。
  cfg.node_name = node.get_name();
  cfg.robot_namespace =
      declareAndGet<std::string>(node, "robot_namespace", cfg.robot_namespace);
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
  cfg.navigation.goal_tolerance_m = declareAndGet<double>(
      node, "navigation.goal_tolerance_m", cfg.navigation.goal_tolerance_m);
  cfg.navigation.server_wait_timeout_s =
      declareAndGet<double>(node, "navigation.server_wait_timeout_s",
                            cfg.navigation.server_wait_timeout_s);

  cfg.motion.action_name = declareAndGet<std::string>(
      node, "motion.action_name", cfg.motion.action_name);
  cfg.motion.joint_names =
      declareAndGet<std::vector<std::string>>(node, "motion.joint_names", cfg.motion.joint_names);
  cfg.motion.default_timeout_s = declareAndGet<double>(
      node, "motion.default_timeout_s", cfg.motion.default_timeout_s);
  cfg.motion.default_max_velocity = declareAndGet<double>(
      node, "motion.default_max_velocity", cfg.motion.default_max_velocity);
  cfg.motion.gripper_joint = declareAndGet<std::string>(
      node, "motion.gripper_joint", cfg.motion.gripper_joint);
  cfg.motion.gripper_open_position = declareAndGet<double>(
      node, "motion.gripper_open_position", cfg.motion.gripper_open_position);
  cfg.motion.gripper_closed_position = declareAndGet<double>(
      node, "motion.gripper_closed_position", cfg.motion.gripper_closed_position);
  // 注意：motion.named_targets 故意不在这里读。
  // 它是 map<string, vector<double>>，而 ROS 2 的参数类型只有
  // bool / int64 / double / string 以及它们的数组 —— 没有 map 类型，
  // 无法用参数表达。只能由代码直接设置。

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
  cfg.perception.queue_depth = declareAndGet<int>(
      node, "perception.queue_depth", cfg.perception.queue_depth);
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

  return cfg;
}

std::string normalizeNamespace(const std::string &ns) {
  if (ns.empty()) {
    return ns;
  }
  std::string out = ns;
  if (out.front() != '/') {
    out.insert(out.begin(), '/'); // "robot1" → "/robot1"
  }
  // 削掉全部尾部斜杠。允许削成空串 —— 只由 '/' 组成的输入表示根命名空间，
  // 而 ROS 2 拒绝 "//"，只接受 "/"（等价于根）。
  while (!out.empty() && out.back() == '/') {
    out.pop_back();
  }
  return out;
}

}  // namespace robot_sdk
