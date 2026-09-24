#include "robot_sdk_sim/mock_robot_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_action/rcl_action.h>

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

// 导航：反馈不必跟控制同频 —— 50 Hz 的控制回路配 10 Hz 的反馈，
// 客户足够看清进度，又不会被刷爆。
constexpr int kFeedbackEveryNTicks = 5;
// 客户没给 max_speed（或给了 <= 0）时用的默认速度。
constexpr double kDefaultMaxSpeedMps = 0.5;

/// 造一个 TF 消息。`parent -> child`，带平移和偏航。
geometry_msgs::msg::TransformStamped makeTransform(const rclcpp::Time &stamp,
                                                   const std::string &parent,
                                                   const std::string &child,
                                                   double x, double y, double z,
                                                   double yaw) {
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

} // namespace

MockRobotNode::MockRobotNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("mock_robot", options) {
  // 配置全部来自 ROS 参数 —— 客户在 launch 文件里覆盖。
  config_ = RobotConfig::fromNode(*this);

  timer_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 关节：优先用配置里的；没配就给一套默认（参考平台是 4 轴臂）。
  joint_names_ = config_.motion.joint_names;
  if (joint_names_.empty()) {
    joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_joint"};
  }
  joint_positions_.assign(joint_names_.size(), 0.0);
  if (!config_.motion.gripper_joint.empty()) {
    ensureJoint(config_.motion.gripper_joint);
  }

  // 话题名一律用**相对名**，由 rcl 按节点命名空间展开。
  status_pub_ = create_publisher<RobotStatus>(config_.status.status_topic, 10);
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      config_.status.joint_states_topic, 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  // 传感器挂载点是**静态变换**：发布一次 → 之后一直在（transient_local）。
  const auto stamp = now();
  const std::string &base = config_.navigation.base_frame;
  static_tf_broadcaster_->sendTransform(
      makeTransform(stamp, base, laser_frame_, 0.1, 0.0, 0.2, 0.0));
  static_tf_broadcaster_->sendTransform(
      makeTransform(stamp, base, imu_frame_, 0.0, 0.0, 0.1, 0.0));
  static_tf_broadcaster_->sendTransform(
      makeTransform(stamp, base, camera_frame_, 0.2, 0.0, 0.3, 0.0));

  // 状态查询服务：按需给一份"此刻"的状态快照。
  status_service_ = create_service<GetRobotStatus>(
      config_.status.get_status_service,
      [this](const std::shared_ptr<GetRobotStatus::Request> request,
             std::shared_ptr<GetRobotStatus::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        const RobotStatus status = makeStatus(lock); // 传锁作凭证
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

  // 导航动作服务端。它跑在**独立的可重入组**里：目标/取消可以在 50 Hz 控制回路
  // 运行期间到达 —— 两条车道真的并发，这正是上面那些状态需要 mutex_ 的理由。
  action_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  nav_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, config_.navigation.action_name,
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const NavigateToPose::Goal> goal) {
        return handleNavGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleNavigate> handle) {
        return handleNavCancel(handle);
      },
      [this](const std::shared_ptr<GoalHandleNavigate> handle) {
        handleNavAccepted(handle);
      },
      rcl_action_server_get_default_options(), action_group_);

  control_timer_ = create_wall_timer(
      std::chrono::duration<double>(kControlPeriodS),
      [this]() { controlTick(); }, timer_group_);
  status_timer_ = create_wall_timer(
      std::chrono::duration<double>(kStatusPeriodS), [this]() { statusTick(); },
      timer_group_);

  RCLCPP_INFO(get_logger(), "Mock robot ready: %zu joints, status topic '%s'",
              joint_names_.size(), config_.status.status_topic.c_str());
}

std::size_t MockRobotNode::ensureJoint(const std::string &name) {
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    if (joint_names_[i] == name) {
      return i;
    }
  }
  joint_names_.push_back(name);
  joint_positions_.push_back(0.0);
  return joint_names_.size() - 1;
}

MockRobotNode::RobotStatus
MockRobotNode::makeStatus(std::lock_guard<std::mutex> & /*lock*/) const {
  // 参数不命名 = 一眼看出它是**凭证**，不是数据。
  // 它不是"自己加锁"，所以这里绝不能再 lock 一次 —— 调用方已经持锁了。
  //
  // 后续轮次加导航/运动时，**只在这里加分支**，状态话题与服务会自动同步。
  RobotStatus status;
  status.header.stamp = now();
  status.header.frame_id = config_.navigation.base_frame;
  if (nav_active_) {
    status.state = RobotStatus::STATE_NAVIGATING;
  } else if (charging_) {
    status.state = RobotStatus::STATE_CHARGING;
  } else {
    status.state = RobotStatus::STATE_IDLE;
  }

  status.battery_percentage = static_cast<float>(battery_);
  status.is_charging = charging_;
  status.is_battery_low = battery_ < kLowBatteryThreshold;
  status.battery_voltage = static_cast<float>(
      kBatteryNominalVoltage + kBatteryVoltagePerPercent * battery_);
  status.uptime_seconds = uptime_s_;

  // 模拟机器人没有故障，也没有急停按钮 —— 显式写零，而不是靠消息默认值。
  status.error_code = 0;
  status.error_message = "";
  status.is_estop_engaged = false;
  return status;
}

void MockRobotNode::navTick() {
  std::shared_ptr<GoalHandleNavigate> handle;
  NavigateToPose::Result::SharedPtr result;
  NavigateToPose::Feedback feedback;
  bool send_feedback = false;
  bool done = false;
  bool canceled = false;

  {
    // ── 锁内：推进数据 + 判定终态 + 组装反馈（不碰框架）──
    std::lock_guard<std::mutex> lock(mutex_);
    if (!nav_active_) {
      return;
    }
    handle = nav_goal_handle_;

    if (nav_cancel_requested_) {
      nav_active_ = false;
      nav_cancel_requested_ = false;
      done = true;
      canceled = true;
      result = std::make_shared<NavigateToPose::Result>();
      result->message = "canceled by client";
    } else {
      const double remaining = distance(pose_, nav_target_);
      if (remaining <= config_.navigation.goal_tolerance_m) {
        pose_ = nav_target_; // 对齐，免得留下毫米级残差
        nav_active_ = false;
        done = true;
        result = std::make_shared<NavigateToPose::Result>();
        result->success = true;
        result->final_distance_error = static_cast<float>(remaining);
        result->message = "reached target";
      } else {
        // 朝目标走一步，并把朝向转向目标（模拟一个会转头的前进底盘）。
        const double step =
            std::min(nav_max_speed_ * kControlPeriodS, remaining);
        pose_.x += step * (nav_target_.x - pose_.x) / remaining;
        pose_.y += step * (nav_target_.y - pose_.y) / remaining;
        pose_.yaw =
            std::atan2(nav_target_.y - pose_.y, nav_target_.x - pose_.x);

        if (++feedback_divider_ >= kFeedbackEveryNTicks) {
          feedback_divider_ = 0;
          send_feedback = true;
          feedback = makeNavFeedback(lock); // ← 传锁作凭证
        }
      }
    }
  } // ← 锁在这里放掉：下面全是框架调用（I/O）

  if (done) {
    if (canceled) {
      handle->canceled(result);
    } else {
      handle->succeed(result);
    }
    return;
  }
  if (send_feedback) {
    handle->publish_feedback(
        std::make_shared<NavigateToPose::Feedback>(feedback));
  }
}

rclcpp_action::GoalResponse
MockRobotNode::handleNavGoal(const rclcpp_action::GoalUUID & /*uuid*/,
                             std::shared_ptr<const NavigateToPose::Goal> goal) {
  // 本模拟器只在全局帧里规划，不做 TF 变换 —— 那是导航后端的事。
  const std::string &frame = goal->target_pose.header.frame_id;
  if (!frame.empty() && frame != config_.navigation.global_frame) {
    RCLCPP_WARN(get_logger(), "拒绝导航目标：帧 '%s' 不是全局帧 '%s'",
                frame.c_str(), config_.navigation.global_frame.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  bool busy = false;
  {
    // 锁内只碰数据：占位 + 记下目标。日志与回复都放到锁外。
    std::lock_guard<std::mutex> lock(mutex_);
    busy = nav_active_;
    if (!busy) {
      nav_active_ = true; // 占位：防止并发到达的第二个目标也被接受
      // 清掉可能残留的取消请求（与取消回调存在竞态，见 学习笔记.md §7.8）。
      nav_cancel_requested_ = false;
      nav_target_ = toPose2D(goal->target_pose);
      nav_max_speed_ =
          goal->max_speed > 0.0f ? goal->max_speed : kDefaultMaxSpeedMps;
      feedback_divider_ = 0;
    }
  }

  if (busy) {
    RCLCPP_WARN(get_logger(), "拒绝导航目标：已经在导航中");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->avoid_obstacles) {
    RCLCPP_WARN_ONCE(get_logger(),
                     "本模拟器没有障碍物模型，avoid_obstacles 被忽略");
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MockRobotNode::handleNavCancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle) {
  // 只有"还在动"的目标才值得取消。（is_active() 在锁外调用。）
  if (!goal_handle->is_active()) {
    return rclcpp_action::CancelResponse::REJECT;
  }

  // 只拨一个 flag，真正的"停下来 + 回 canceled()"交给控制回路。
  // 为什么不直接调 goal_handle->is_canceling()：见 学习笔记.md §7.8。
  std::lock_guard<std::mutex> lock(mutex_);
  nav_cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MockRobotNode::handleNavAccepted(
    const std::shared_ptr<GoalHandleNavigate> goal_handle) {
  Pose2D target;
  double speed = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    nav_goal_handle_ = goal_handle;
    target = nav_target_;
    speed = nav_max_speed_;
  }
  RCLCPP_INFO(get_logger(), "开始导航 → (%.2f, %.2f)，最大速度 %.2f m/s",
              target.x, target.y, speed);
}

MockRobotNode::NavigateToPose::Feedback
MockRobotNode::makeNavFeedback(std::lock_guard<std::mutex> & /*lock*/) const {
  // 参数不命名 = 凭证（同 makeStatus）。
  NavigateToPose::Feedback feedback;
  feedback.current_pose.header.stamp = now();
  feedback.current_pose.header.frame_id = config_.navigation.global_frame;
  feedback.current_pose.pose = toPoseMsg(pose_);
  feedback.distance_remaining =
      static_cast<float>(distance(pose_, nav_target_));
  feedback.estimated_time_remaining =
      nav_max_speed_ > 0.0
          ? static_cast<float>(feedback.distance_remaining / nav_max_speed_)
          : 0.0f;
  feedback.current_state = "navigating";
  return feedback;
}

void MockRobotNode::controlTick() {
  navTick(); // 先推进导航，再发布"机器人现在长什么样"
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

    status = makeStatus(lock); // ← 传锁作凭证
  } // ← 锁在这里放掉

  status_pub_->publish(status); // 锁外做 I/O
}

void MockRobotNode::publishFrames() {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto stamp = now();
  // 这里是模拟机器人的"真值"：它假装 odom 与 global_frame 重合，
  // 于是把位姿直接发成 odom -> base 的变换。
  tf_broadcaster_->sendTransform(makeTransform(
      stamp, config_.navigation.global_frame, "odom", 0.0, 0.0, 0.0, 0.0));
  tf_broadcaster_->sendTransform(
      makeTransform(stamp, "odom", config_.navigation.base_frame, pose_.x,
                    pose_.y, 0.0, pose_.yaw));
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

} // namespace robot_sdk::sim
