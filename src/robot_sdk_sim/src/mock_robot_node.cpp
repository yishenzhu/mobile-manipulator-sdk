#include "robot_sdk_sim/mock_robot_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

// 关节运动的到位容差（弧度 / 米），以及客户没给 max_velocity 时的默认值。
constexpr double kJointTolerance = 0.01;
constexpr double kDefaultJointVelocity = 0.5;

// 传感器：控制回路 50 Hz → 传感器 10 Hz。
constexpr int kSensorEveryNTicks = 5;
// 激光：一圈 1° 分辨率，量程照常见 2D 雷达填。
constexpr int kScanRays = 360;
constexpr double kScanRangeMin = 0.05;
constexpr double kScanRangeMax = 10.0;
// 激光挂载点相对 base 的 x 偏移 —— **必须与构造函数里那条静态 TF 一致**。
constexpr double kLaserOffsetX = 0.1;

// 模拟机器人的"世界"：一个矩形房间，**四面墙就是它唯一的障碍物**。
// 它没有障碍物模型（见 docs/mock-robot-node.md §1.3），所以扫描得在这里"编"出来。
constexpr double kRoomHalfWidth = 5.0;  // x ∈ [-5, 5]
constexpr double kRoomHalfHeight = 4.0; // y ∈ [-4, 4]

/// 从 (px, py) 沿角度 a 射出的光线，打到矩形房间墙上的距离。
///
/// 分别求与 x = ±半宽、y = ±半高 四个面的交点，取最近的那个。
double rayToWall(double px, double py, double a) {
  const double dx = std::cos(a);
  const double dy = std::sin(a);
  // 平行于某个轴时（dx 或 dy 为 0）那个面永远打不到 —— 用 inf 表示。
  const double tx = dx > 0.0 ? (kRoomHalfWidth - px) / dx
                             : (dx < 0.0 ? (-kRoomHalfWidth - px) / dx
                                         : std::numeric_limits<double>::infinity());
  const double ty = dy > 0.0 ? (kRoomHalfHeight - py) / dy
                             : (dy < 0.0 ? (-kRoomHalfHeight - py) / dy
                                         : std::numeric_limits<double>::infinity());
  return std::min(tx, ty);
}

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

  // 传感器用与 SDK 侧 `PerceptionAPI` **完全相同的 QoS**：
  // best_effort + keep_last(队列深度)。两边不一致的话订阅会**静默收不到**数据。
  auto sensor_qos = rclcpp::SensorDataQoS();
  sensor_qos.keep_last(static_cast<std::size_t>(config_.perception.queue_depth));
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      config_.perception.laser_scan_topic, sensor_qos);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(config_.perception.imu_topic,
                                                     sensor_qos);

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
  // 它跑在**独立的互斥组**里 —— 不跟 50 Hz 控制回路抢同一条车道。
  // （放在 timer_group_ 里也能工作，但实测会让控制回路的抖动变差，见 学习笔记.md §7.9。）
  service_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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
      rclcpp::ServicesQoS(), service_group_);

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

  // 关节运动服务端：与导航**共用** action_group_（同一条可重入车道）。
  motion_server_ = rclcpp_action::create_server<MoveToJointPositions>(
      this, config_.motion.action_name,
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const MoveToJointPositions::Goal> goal) {
        return handleMotionGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleMove> handle) {
        return handleMotionCancel(handle);
      },
      [this](const std::shared_ptr<GoalHandleMove> handle) {
        handleMotionAccepted(handle);
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
  if (nav_.running()) {
    status.state = RobotStatus::STATE_NAVIGATING;
  } else if (motion_.running()) {
    status.state = RobotStatus::STATE_MOVING;
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
  // 骨架负责加锁/空转/取消/终态回调/反馈发布；这里只写"这一拍做什么"。
  tickAction<NavigateToPose>(
      mutex_, nav_,
      [this](std::lock_guard<std::mutex> &lock, auto &result, auto &feedback) {
        const double remaining = distance(pose_, nav_target_);
        if (remaining <= config_.navigation.goal_tolerance_m) {
          pose_ = nav_target_; // 对齐，免得留下毫米级残差
          result = std::make_shared<NavigateToPose::Result>();
          result->success = true;
          result->final_distance_error = static_cast<float>(remaining);
          result->message = "reached target";
          return; // 返回非空 result 即"完成"（骨架会收尾）
        }

        // 朝目标走一步，并把朝向转向目标（模拟一个会转头的前进底盘）。
        const double step = std::min(nav_max_speed_ * kControlPeriodS, remaining);
        pose_.x += step * (nav_target_.x - pose_.x) / remaining;
        pose_.y += step * (nav_target_.y - pose_.y) / remaining;
        pose_.yaw = std::atan2(nav_target_.y - pose_.y, nav_target_.x - pose_.x);

        if (++nav_.feedback_divider >= kFeedbackEveryNTicks) {
          nav_.feedback_divider = 0;
          feedback = std::make_shared<NavigateToPose::Feedback>(makeNavFeedback(lock));
        }
      });
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
    busy = nav_.busy();
    if (!busy) {
      // reserve() 只置"占位"相位，**不置 kRunning** —— 句柄要等 accepted 回调才到，
      // 而 kRunning 一旦成立，navTick 就会拿句柄去回调框架，那个窗口里它还是空的。
      // 它顺手清掉残留的取消请求（与取消回调存在竞态，见 学习笔记.md §7.8）。
      nav_.reserve();
      nav_target_ = toPose2D(goal->target_pose);
      nav_max_speed_ =
          goal->max_speed > 0.0f ? goal->max_speed : kDefaultMaxSpeedMps;
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

  // 只投一位信箱，真正的"停下来 + 回 canceled()"交给控制回路。
  // 为什么不直接调 goal_handle->is_canceling()：见 学习笔记.md §7.8。
  std::lock_guard<std::mutex> lock(mutex_);
  nav_.cancel_requested = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MockRobotNode::handleNavAccepted(
    const std::shared_ptr<GoalHandleNavigate> goal_handle) {
  Pose2D target;
  double speed = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 句柄与相位在**同一个临界区**里改变 → kRunning 蕴含 handle 非空
    // （见 action_slot.hpp）：不靠时序，靠构造。
    nav_.begin(goal_handle);
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

MockRobotNode::MoveToJointPositions::Feedback MockRobotNode::makeMotionFeedback(
    std::lock_guard<std::mutex> & /*lock*/) const {
  // 参数不命名 = 凭证（同 makeStatus）。
  MoveToJointPositions::Feedback feedback;
  feedback.current.header.stamp = now();
  feedback.current.name = joint_names_;
  feedback.current.position = joint_positions_;

  double worst = 0.0;
  for (std::size_t i = 0; i < joint_positions_.size(); ++i) {
    worst = std::max(worst, std::abs(motion_target_[i] - joint_positions_[i]));
  }
  // 完成度 = 从"起始最大误差"缩到"当前最大误差"的比例；起始就是 0 视为已完成。
  feedback.completion_percentage =
      motion_initial_error_ > 0.0
          ? static_cast<float>(100.0 * (1.0 - worst / motion_initial_error_))
          : 100.0f;
  return feedback;
}

void MockRobotNode::motionTick() {
  // 同 navTick：骨架一样，只有中间这段"推进"不同。
  tickAction<MoveToJointPositions>(
      mutex_, motion_,
      [this](std::lock_guard<std::mutex> &lock, auto &result, auto &feedback) {
        // 每个关节各自朝目标走一步。`|误差| <= 本拍行程` 时直接对齐 ——
        // 既避免过冲，也让"误差恰好为 0"不会把关节推离目标。
        const double step = motion_max_velocity_ * kControlPeriodS;
        double worst = 0.0;
        for (std::size_t i = 0; i < joint_positions_.size(); ++i) {
          const double error = motion_target_[i] - joint_positions_[i];
          if (std::abs(error) <= step) {
            joint_positions_[i] = motion_target_[i];
          } else {
            joint_positions_[i] += std::copysign(step, error);
          }
          worst = std::max(worst, std::abs(motion_target_[i] - joint_positions_[i]));
        }

        if (worst <= kJointTolerance) {
          joint_positions_ = motion_target_; // 对齐，免得留下残差
          result = std::make_shared<MoveToJointPositions::Result>();
          result->success = true;
          result->final_position_error = static_cast<float>(worst);
          result->message = "reached target";
        } else if (++motion_.feedback_divider >= kFeedbackEveryNTicks) {
          // 反馈不必跟控制同频（同导航）：50 Hz 控制、10 Hz 反馈。
          motion_.feedback_divider = 0;
          feedback =
              std::make_shared<MoveToJointPositions::Feedback>(makeMotionFeedback(lock));
        }
      });
}

rclcpp_action::GoalResponse MockRobotNode::handleMotionGoal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const MoveToJointPositions::Goal> goal) {
  // 关节名与位置必须一一对应 —— 否则根本不知道该动哪个关节。
  if (goal->target.name.empty() ||
      goal->target.name.size() != goal->target.position.size()) {
    RCLCPP_WARN(get_logger(), "拒绝运动目标：关节名与位置数量不匹配");
    return rclcpp_action::GoalResponse::REJECT;
  }

  bool busy = false;
  bool unknown = false;
  {
    // 锁内只碰数据：先在**局部**把目标拼好，全部合法才提交（不留半成品状态）。
    std::lock_guard<std::mutex> lock(mutex_);
    busy = motion_.busy();
    if (!busy) {
      std::vector<double> target = joint_positions_; // 目标没点名的关节保持原位
      for (std::size_t k = 0; k < goal->target.name.size(); ++k) {
        const auto it =
            std::find(joint_names_.begin(), joint_names_.end(), goal->target.name[k]);
        if (it == joint_names_.end()) {
          unknown = true;
          break;
        }
        target[static_cast<std::size_t>(it - joint_names_.begin())] =
            goal->target.position[k];
      }
      if (!unknown) {
        motion_target_ = std::move(target);
        motion_max_velocity_ =
            goal->max_velocity > 0.0f ? goal->max_velocity : kDefaultJointVelocity;
        // reserve() 只置"占位"相位（理由同 handleNavGoal），
        // 并顺手清掉残留的取消请求、复位反馈降频。
        motion_.reserve();
        // 起始最大误差 —— completion_percentage 的分母。
        motion_initial_error_ = 0.0;
        for (std::size_t i = 0; i < motion_target_.size(); ++i) {
          motion_initial_error_ = std::max(
              motion_initial_error_, std::abs(motion_target_[i] - joint_positions_[i]));
        }
      }
    }
  }

  if (busy) {
    RCLCPP_WARN(get_logger(), "拒绝运动目标：已经在运动");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (unknown) {
    RCLCPP_WARN(get_logger(), "拒绝运动目标：包含本机没有的关节");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MockRobotNode::handleMotionCancel(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
  // 同 handleNavCancel：只拨 flag，真正的"停下 + 回 canceled()"交给控制回路。
  if (!goal_handle->is_active()) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  motion_.cancel_requested = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MockRobotNode::handleMotionAccepted(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  motion_.begin(goal_handle); // 句柄与相位同时改变（理由同 handleNavAccepted）
}

void MockRobotNode::controlTick() {
  navTick();      // 先推进导航，
  motionTick();   // 再推进关节运动，
  publishJoints();
  publishFrames();
  publishSensors(); // 内部降频到 10 Hz —— 传感器不必跟 50 Hz 控制同频
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
  Pose2D pose;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose = pose_;
  }
  // 锁外做 I/O —— sendTransform 会走 DDS，不该占着锁（同 publishJoints）。
  // 这里是模拟机器人的"真值"：它假装 odom 与 global_frame 重合，
  // 于是把位姿直接发成 odom -> base 的变换。
  const auto stamp = now();
  tf_broadcaster_->sendTransform(makeTransform(
      stamp, config_.navigation.global_frame, "odom", 0.0, 0.0, 0.0, 0.0));
  tf_broadcaster_->sendTransform(
      makeTransform(stamp, "odom", config_.navigation.base_frame, pose.x, pose.y, 0.0,
                    pose.yaw));
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

void MockRobotNode::publishSensors() {
  sensor_msgs::msg::LaserScan scan;
  sensor_msgs::msg::Imu imu;

  {
    // ── 锁内：降频判断 + 取数 + 组装（不碰发布）──
    std::lock_guard<std::mutex> lock(mutex_);

    // 降频判断其实**可以**放在锁外：`sensor_divider_` 只有控制回路这一条车道会碰
    // （`controlTick` 全在 timer_group_ 的互斥组里，不会和自己并发），严格说不需要锁。
    // 仍然放在锁内，是为了守住一条更简单的规则：
    //   **`mutex_` 下面那一块成员，全部在锁内访问，没有例外。**
    // 挪出去省下的是一次无争用的 lock/unlock（纳秒级，量不出来），
    // 代价却是引入一个"这个成员其实不受保护"的例外 —— 以后有人在别的车道
    // 顺手读它，就是**静默的数据竞争**。判据同 §4.5 / §6.3：不为量不到的收益加规则。
    // （`last_yaw_` 是同一类。）
    if (++sensor_divider_ < kSensorEveryNTicks) {
      return; // 还没到下一拍
    }
    sensor_divider_ = 0;

    const auto stamp = now();

    // ── 激光：从挂载点朝一圈方向射线，打到房间的墙上 ──
    scan.header.stamp = stamp;
    scan.header.frame_id = laser_frame_;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = 2.0 * M_PI / kScanRays;
    scan.range_min = kScanRangeMin;
    scan.range_max = kScanRangeMax;
    scan.scan_time = static_cast<float>(kControlPeriodS * kSensorEveryNTicks);

    // 挂载点：相对 base 的偏移，随机器人朝向一起转。
    const double laser_x = pose_.x + kLaserOffsetX * std::cos(pose_.yaw);
    const double laser_y = pose_.y + kLaserOffsetX * std::sin(pose_.yaw);

    scan.ranges.reserve(kScanRays);
    for (int i = 0; i < kScanRays; ++i) {
      // angle 是**激光坐标系**里的角 —— 机器人转了，世界角就跟着转。
      const double angle = scan.angle_min + i * scan.angle_increment;
      const double range = rayToWall(laser_x, laser_y, pose_.yaw + angle);
      // 超出量程 = 没有回波。ROS 约定用 inf，不是 0、也不是 range_max。
      scan.ranges.push_back(range <= kScanRangeMax
                                ? static_cast<float>(std::max(range, kScanRangeMin))
                                : std::numeric_limits<float>::infinity());
    }

    // ── IMU：朝向取真值；角速度只能由上一拍的偏航差得到，没有别的来源 ──
    imu.header.stamp = stamp;
    imu.header.frame_id = imu_frame_;
    imu.orientation = quaternionFromYaw(pose_.yaw);
    imu.angular_velocity.z =
        angleDifference(pose_.yaw, last_yaw_) / (kControlPeriodS * kSensorEveryNTicks);
    last_yaw_ = pose_.yaw;
    // 静止时加速度计读到的是**支持力**，所以 z 是 +g 而不是 -g。
    imu.linear_acceleration.z = 9.81;
    // 协方差给个小值，不给 0：0 在 ROS 里表示"完全已知"，
    // 下游滤波器会当特例处理（有的直接拿它做除法）。本模拟器无噪声，
    // 但也没必要**假装**完全已知。
    imu.orientation_covariance[0] = 1e-4;
    imu.angular_velocity_covariance[0] = 1e-4;
    imu.linear_acceleration_covariance[0] = 1e-4;
  } // ← 锁在这里放掉

  scan_pub_->publish(scan); // 锁外做 I/O
  imu_pub_->publish(imu);
}

} // namespace robot_sdk::sim
