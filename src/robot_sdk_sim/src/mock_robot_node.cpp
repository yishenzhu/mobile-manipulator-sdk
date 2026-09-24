#include "robot_sdk_sim/mock_robot_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace robot_sdk::sim {
namespace {

// 三个定时器的周期：控制最密（50 Hz），状态次之（10 Hz）。
constexpr double kControlPeriodS = 0.02;
constexpr double kStatusPeriodS = 0.1;

// 电量模拟：不充电时缓慢下降，低于 10% 自动进入充电，充满后停止。
constexpr double kBatteryDrainPerTick = 0.05;
constexpr double kBatteryChargePerTick = 0.5;
constexpr double kLowBatteryThreshold = 20.0;

// 电压模型：空电到满电之间线性变化（只是给状态字段一个像样的值）。
constexpr double kBatteryNominalVoltage = 22.2;
constexpr double kBatteryVoltagePerPercent = 0.05;

/// 造一个 TF 消息。`parent -> child`，带平移和偏航。
geometry_msgs::msg::TransformStamped makeTransform(const rclcpp::Time& stamp,
                                                   const std::string& parent,
                                                   const std::string& child, double x, double y,
                                                   double z, double yaw) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = z;
  tf.transform.rotation = quaternionFromYaw(yaw);
  return tf;
}

}  // namespace

MockRobotNode::MockRobotNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mock_robot", options) {
  // 配置全部来自 ROS 参数 —— 客户在 launch 文件里覆盖。
  config_ = RobotConfig::fromNode(*this);

  timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 关节：优先用配置里的；没配就给一套默认（参考平台是 4 轴臂）。
  joint_names_ = config_.motion.joint_names;
  if (joint_names_.empty()) {
    joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"};
  }
  joint_positions_.assign(joint_names_.size(), 0.0);
  if (!config_.motion.gripper_joint.empty()) {
    ensureJoint(config_.motion.gripper_joint);
  }

  // 话题名一律用**相对名**，由 rcl 按节点命名空间展开。
  status_pub_ = create_publisher<RobotStatus>(config_.status.status_topic, 10);
  joint_pub_ =
      create_publisher<sensor_msgs::msg::JointState>(config_.status.joint_states_topic, 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  // 传感器挂载点是**静态变换**：发布一次 → 之后一直在（transient_local）。
  const auto stamp = now();
  const std::string& base = config_.navigation.base_frame;
  static_tf_broadcaster_->sendTransform(
      makeTransform(stamp, base, laser_frame_, 0.1, 0.0, 0.2, 0.0));
  static_tf_broadcaster_->sendTransform(makeTransform(stamp, base, imu_frame_, 0.0, 0.0, 0.1, 0.0));
  static_tf_broadcaster_->sendTransform(
      makeTransform(stamp, base, camera_frame_, 0.2, 0.0, 0.3, 0.0));

  // 状态查询服务：按需给一份"此刻"的状态快照。
  status_service_ = create_service<GetRobotStatus>(
      config_.status.get_status_service,
      [this](const std::shared_ptr<GetRobotStatus::Request> request,
             std::shared_ptr<GetRobotStatus::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        const RobotStatus status = makeStatus(lock);  // 传锁作凭证
        response->success = true;
        response->message = "ok";
        response->status = status;
        if (request->include_joint_states) {
          response->joint_states.header.stamp = status.header.stamp;
          response->joint_states.name = joint_names_;
          response->joint_states.position = joint_positions_;
          response->joint_states.velocity.assign(joint_names_.size(), 0.0);
          response->joint_states.effort.assign(joint_names_.size(), 0.0);
        }
      },
      rclcpp::ServicesQoS(), timer_group_);

  control_timer_ = create_wall_timer(
      std::chrono::duration<double>(kControlPeriodS), [this]() { controlTick(); }, timer_group_);
  status_timer_ = create_wall_timer(
      std::chrono::duration<double>(kStatusPeriodS), [this]() { statusTick(); }, timer_group_);

  RCLCPP_INFO(get_logger(), "Mock robot ready: %zu joints, status topic '%s'",
              joint_names_.size(), config_.status.status_topic.c_str());
}

std::size_t MockRobotNode::ensureJoint(const std::string& name) {
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    if (joint_names_[i] == name) {
      return i;
    }
  }
  joint_names_.push_back(name);
  joint_positions_.push_back(0.0);
  return joint_names_.size() - 1;
}

MockRobotNode::RobotStatus MockRobotNode::makeStatus(std::lock_guard<std::mutex>& /*lock*/) const {
  // 参数不命名 = 一眼看出它是**凭证**，不是数据。
  // 它不是"自己加锁"，所以这里绝不能再 lock 一次 —— 调用方已经持锁了。
  //
  // 后续轮次加导航/运动时，**只在这里加分支**，状态话题与服务会自动同步。
  RobotStatus status;
  status.header.stamp = now();
  status.header.frame_id = config_.navigation.base_frame;
  status.state = charging_ ? RobotStatus::STATE_CHARGING : RobotStatus::STATE_IDLE;

  status.battery_percentage = static_cast<float>(battery_);
  status.is_charging = charging_;
  status.is_battery_low = battery_ < kLowBatteryThreshold;
  status.battery_voltage =
      static_cast<float>(kBatteryNominalVoltage + kBatteryVoltagePerPercent * battery_);
  status.uptime_seconds = uptime_s_;

  // 模拟机器人没有故障，也没有急停按钮 —— 显式写零，而不是靠消息默认值。
  status.error_code = 0;
  status.error_message = "";
  status.is_estop_engaged = false;
  return status;
}

void MockRobotNode::controlTick() {
  // 目前只发布"机器人现在长什么样"。导航/运动的推进逻辑后续加入。
  publishJoints();
  publishFrames();
}

void MockRobotNode::statusTick() {
  RobotStatus status;
  {
    // 锁内：推进数据 + 组装（不碰发布）
    std::lock_guard<std::mutex> lock(mutex_);
    uptime_s_ += kStatusPeriodS;

    if (charging_) {
      battery_ = std::min(100.0, battery_ + kBatteryChargePerTick);
      if (battery_ >= 100.0) {
        charging_ = false;
      }
    } else {
      battery_ = std::max(0.0, battery_ - kBatteryDrainPerTick);
      if (battery_ <= 10.0) {
        charging_ = true;
      }
    }

    status = makeStatus(lock);  // ← 传锁作凭证
  }  // ← 锁在这里放掉

  status_pub_->publish(status);  // 锁外做 I/O
}

void MockRobotNode::publishFrames() {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto stamp = now();
  // 这里是模拟机器人的"真值"：它假装 odom 与 global_frame 重合，
  // 于是把位姿直接发成 odom -> base 的变换。
  tf_broadcaster_->sendTransform(
      makeTransform(stamp, config_.navigation.global_frame, "odom", 0.0, 0.0, 0.0, 0.0));
  tf_broadcaster_->sendTransform(makeTransform(stamp, "odom", config_.navigation.base_frame,
                                              pose_.x, pose_.y, 0.0, pose_.yaw));
}

void MockRobotNode::publishJoints() {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.name = joint_names_;
    msg.position = joint_positions_;
    msg.velocity.assign(joint_names_.size(), 0.0);
    msg.effort.assign(joint_names_.size(), 0.0);
  }
  joint_pub_->publish(msg);
}

}  // namespace robot_sdk::sim
