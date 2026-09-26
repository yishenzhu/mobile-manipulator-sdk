#include "robot_sdk_sim/mock_robot_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
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
// 激光挂载点相对 base 的偏移 —— **必须与构造函数里那条静态 TF 一致**。
constexpr double kLaserOffsetX = 0.1;
constexpr double kLaserOffsetZ = 0.2; // 激光是水平的，z 不参与计算；写对是为了诚实

// 模拟机器人的"世界"：一个长方体房间，**六面就是它唯一的障碍物**。
// 它没有障碍物模型（见 docs/mock-robot-node.md §1.3），所以扫描得在这里"编"出来。
constexpr double kRoomHalfWidth = 5.0;  // x ∈ [-5, 5]
constexpr double kRoomHalfHeight = 4.0; // y ∈ [-4, 4]
// 房间是有高度的：地面 z=0、天花板 z=2.5。
// 2D 激光打不到它们（射线是水平的），但深度相机能 —— 所以相机需要三维的房间。
constexpr double kRoomFloorZ = 0.0;
constexpr double kRoomCeilingZ = 2.5;

// 深度相机：尺寸小就够 —— 它是让客户"有东西可读"，不是图像质量展示。
constexpr int kDepthWidth = 64;
constexpr int kDepthHeight = 48;
constexpr double kCameraHfovRad = M_PI / 3.0; // 水平视场 60°
// 相机安装点（**必须与构造函数里那条静态 TF 一致**）：base 前 0.2、上 0.3。
constexpr double kCameraOffsetX = 0.2;
constexpr double kCameraOffsetZ = 0.3;

/// 从 (ox,oy,oz) 沿**单位**方向 (dx,dy,dz) 射出，打在长方体房间上的距离。
/// 打不到返回 inf。
///
/// 这是标准的 **slab 法**：把长方体看成三对平行板，每对给出一个参数区间
/// `[t1, t2]`，三者求交 —— 交出来的下界就是"进入盒子"的距离。
///
/// **激光和相机共用它。** 激光的 dz ≡ 0，所以地面/天花板那两项永远命中不了，
/// 结果和"只看四面墙"完全一样；但共用一份房间模型意味着**不会改了一个忘另一个**。
double rayToBox(double ox, double oy, double oz, double dx, double dy, double dz) {
  const double origin[3] = {ox, oy, oz};
  const double dir[3] = {dx, dy, dz};
  const double lo[3] = {-kRoomHalfWidth, -kRoomHalfHeight, kRoomFloorZ};
  const double hi[3] = {kRoomHalfWidth, kRoomHalfHeight, kRoomCeilingZ};

  double t_enter = -std::numeric_limits<double>::infinity();
  double t_exit = std::numeric_limits<double>::infinity();

  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(dir[axis]) < 1e-12) {
      // 平行于这对板：不在两板之间就永远打不到。
      if (origin[axis] < lo[axis] || origin[axis] > hi[axis]) {
        return std::numeric_limits<double>::infinity();
      }
      continue;
    }
    double t1 = (lo[axis] - origin[axis]) / dir[axis];
    double t2 = (hi[axis] - origin[axis]) / dir[axis];
    if (t1 > t2) {
      std::swap(t1, t2); // 方向为负时近/远会颠倒
    }
    t_enter = std::max(t_enter, t1);
    t_exit = std::min(t_exit, t2);
  }

  // 区间空了（或整个盒子在身后）= 这条射线打不到墙。
  if (t_enter > t_exit || t_exit < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  // ⚠️ 这里是关键：**相机在房间内部**，所以 t_enter 是负的（"入口在身后"），
  //    真正的墙是**出口** t_exit。
  //    一开始把 t_enter 初始成 0 是错的 —— max(0, 负数) 会把负号吃掉，
  //    于是永远返回 0（一整张深度图全是 0，而且不报错）。
  return t_enter > 0.0 ? t_enter : t_exit;
}

/// 深度相机的焦距（像素）—— 由"水平视场 60°"推出来，而不是随手写个数。
/// 像素是正方形的，所以 fx = fy。
double cameraFocalLength() {
  return (kDepthWidth / 2.0) / std::tan(kCameraHfovRad / 2.0);
}

/// 读深度图的第 `index` 个像素（encoding = "32FC1"，每像素一个 float）。
///
/// 用 `memcpy` 而不是把 `data` 强转成 `float*`：后者依赖"vector<uint8_t> 的缓冲区
/// 满足 float 对齐"这个实践中成立、标准里不保证的假设。
float readDepthPixel(const sensor_msgs::msg::Image &image, std::size_t index) {
  float value = 0.0F;
  std::memcpy(&value, image.data.data() + index * sizeof(float), sizeof(float));
  return value;
}

/// 造一个 TF 消息。`parent -> child`，带平移和 **RPY**。
/// 需要 RPY 而不是单个偏航的场合只有一处：相机的**光学帧**（见构造函数）。
geometry_msgs::msg::TransformStamped makeTransformRpy(
    const rclcpp::Time &stamp, const std::string &parent, const std::string &child,
    double x, double y, double z, double roll, double pitch, double yaw) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  return tf;
}

/// 只要平移 + 偏航时的简写（绝大多数挂载点都只用得上这个）。
geometry_msgs::msg::TransformStamped makeTransform(const rclcpp::Time &stamp,
                                                   const std::string &parent,
                                                   const std::string &child, double x,
                                                   double y, double z, double yaw) {
  return makeTransformRpy(stamp, parent, child, x, y, z, 0.0, 0.0, yaw);
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
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      config_.perception.camera_info_topic, sensor_qos);
  depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
      config_.perception.depth_image_topic, sensor_qos);
  point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      config_.perception.point_cloud_topic, sensor_qos);

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

  // 相机的**光学帧**：ROS 约定图像数据的坐标系是 z 朝前 / x 朝右 / y 朝下，
  // 而 body 系是 x 前 / y 左 / z 上 —— 两者差一个固定旋转 RPY(-90°, 0, -90°)。
  // 少广播这一帧，客户拿 TF 去投影深度图会得到**错误但看起来合理**的结果
  // （这正是本项目最怕的那类 bug：不报错）。
  static_tf_broadcaster_->sendTransform(makeTransformRpy(
      stamp, camera_frame_, camera_optical_frame_, 0.0, 0.0, 0.0, -M_PI / 2.0, 0.0,
      -M_PI / 2.0));

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
  sensor_msgs::msg::CameraInfo camera_info;
  sensor_msgs::msg::Image depth;

  {
    // ── 锁内：降频 + 组装（不碰发布）──
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
    scan = makeScan(lock, stamp);
    // 角速度要拿"上一拍"差分，所以先把旧值传进去，**再**更新它。
    imu = makeImu(lock, stamp, last_yaw_);
    last_yaw_ = pose_.yaw;
    camera_info = makeCameraInfo(stamp);
    depth = makeDepthImage(lock, stamp);
  } // ← 锁在这里放掉

  scan_pub_->publish(scan); // 锁外做 I/O
  imu_pub_->publish(imu);
  camera_info_pub_->publish(camera_info);
  depth_pub_->publish(depth);
  // 点云是**从深度图算出来的**，所以这里放在锁外、直接吃已经算好的那一张 ——
  // 不会把同一批射线再打一遍，也不需要再碰共享状态。
  point_cloud_pub_->publish(makePointCloud(depth));
}

sensor_msgs::msg::LaserScan
MockRobotNode::makeScan(std::lock_guard<std::mutex> & /*lock*/,
                        const rclcpp::Time &stamp) const {
  // ── 激光：从挂载点朝一圈方向射线，打到房间的墙上 ──
  sensor_msgs::msg::LaserScan scan;
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
    const double world_angle = pose_.yaw + angle;
    // 激光射线是**水平的**（dz = 0），所以地面/天花板那两项永远命中不了。
    // 与相机共用同一个 `rayToBox` —— 一份房间模型，不会改了一个忘另一个。
    const double range = rayToBox(laser_x, laser_y, kLaserOffsetZ,
                                  std::cos(world_angle), std::sin(world_angle), 0.0);
    // 超出量程 = 没有回波。ROS 约定用 inf，不是 0、也不是 range_max。
    scan.ranges.push_back(range <= kScanRangeMax
                              ? static_cast<float>(std::max(range, kScanRangeMin))
                              : std::numeric_limits<float>::infinity());
  }
  return scan;
}

sensor_msgs::msg::Imu MockRobotNode::makeImu(std::lock_guard<std::mutex> & /*lock*/,
                                             const rclcpp::Time &stamp,
                                             double previous_yaw) const {
  // ── IMU：朝向取真值；角速度只能由上一拍的偏航差得到，没有别的来源 ──
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = imu_frame_;
  imu.orientation = quaternionFromYaw(pose_.yaw);
  imu.angular_velocity.z =
      angleDifference(pose_.yaw, previous_yaw) / (kControlPeriodS * kSensorEveryNTicks);
  // 静止时加速度计读到的是**支持力**，所以 z 是 +g 而不是 -g。
  imu.linear_acceleration.z = 9.81;
  // 协方差给个小值，不给 0：0 在 ROS 里表示"完全已知"，
  // 下游滤波器会当特例处理（有的直接拿它做除法）。本模拟器无噪声，
  // 但也没必要**假装**完全已知。
  imu.orientation_covariance[0] = 1e-4;
  imu.angular_velocity_covariance[0] = 1e-4;
  imu.linear_acceleration_covariance[0] = 1e-4;
  return imu;
}

sensor_msgs::msg::CameraInfo
MockRobotNode::makeCameraInfo(const rclcpp::Time &stamp) const {
  // 内参完全由"尺寸 + 水平视场"决定，**不读任何共享状态** —— 所以不需要凭证。
  // 理想镜头：畸变全 0 ⇒ R 是单位阵、P 就是 K 右下补一列。
  const double fx = cameraFocalLength();
  const double cx = kDepthWidth / 2.0;
  const double cy = kDepthHeight / 2.0;

  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = camera_optical_frame_;
  info.width = kDepthWidth;
  info.height = kDepthHeight;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  info.k = {fx, 0.0, cx, 0.0, fx, cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {fx, 0.0, cx, 0.0, 0.0, fx, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

sensor_msgs::msg::Image
MockRobotNode::makeDepthImage(std::lock_guard<std::mutex> & /*lock*/,
                              const rclcpp::Time &stamp) const {
  const double fx = cameraFocalLength();
  const double cx = kDepthWidth / 2.0;
  const double cy = kDepthHeight / 2.0;

  // 相机安装点：相对 base 的 (0.2, 0, 0.3)，随机器人朝向一起转。
  const double cam_x = pose_.x + kCameraOffsetX * std::cos(pose_.yaw);
  const double cam_y = pose_.y + kCameraOffsetX * std::sin(pose_.yaw);
  const double cam_z = kCameraOffsetZ;
  const double cos_yaw = std::cos(pose_.yaw);
  const double sin_yaw = std::sin(pose_.yaw);

  std::vector<float> meters(static_cast<std::size_t>(kDepthWidth) * kDepthHeight);
  for (int v = 0; v < kDepthHeight; ++v) {
    for (int u = 0; u < kDepthWidth; ++u) {
      // 光学系下的射线：z 朝前。用 (u - cx) 而不是 (u + 0.5 - cx)，
      // 这样 u == cx 那一列**正好落在光轴上** —— 验收要拿它和激光对齐。
      const double ox = (u - cx) / fx;
      const double oy = (v - cy) / fx;
      const double oz = 1.0;
      const double norm = std::sqrt(ox * ox + oy * oy + oz * oz);

      // 光学 (x 右, y 下, z 前) → body (x 前, y 左, z 上)：
      //     body_x = z,   body_y = -x,   body_z = -y
      // 这一步对应上面那条 RPY(-90°, 0, -90°) 的静态 TF —— 两者必须配套，
      // 否则发出去的深度图和 TF 各说各话（而且都不报错）。
      const double bx = oz / norm;
      const double by = -ox / norm;
      const double bz = -oy / norm;
      // 再按机器人偏航转到世界系。
      const double wx = bx * cos_yaw - by * sin_yaw;
      const double wy = bx * sin_yaw + by * cos_yaw;

      // rayToBox 要求方向是**单位向量**，这样 t 才等于米。
      const double t = rayToBox(cam_x, cam_y, cam_z, wx, wy, bz);
      // "32FC1" 存的是**沿光轴的 z 深度**，不是射线长度。
      // 方向是单位的，所以 z 深度 = t ×（该方向在光轴上的分量）= t × bx。
      meters[static_cast<std::size_t>(v) * kDepthWidth + u] =
          std::isinf(t) ? std::numeric_limits<float>::infinity()
                        : static_cast<float>(t * bx);
    }
  }

  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = camera_optical_frame_;
  image.height = kDepthHeight;
  image.width = kDepthWidth;
  image.encoding = "32FC1"; // 每像素一个 float32，单位米
  image.is_bigendian = 0;
  image.step = static_cast<uint32_t>(kDepthWidth * sizeof(float));
  image.data.assign(reinterpret_cast<const uint8_t *>(meters.data()),
                    reinterpret_cast<const uint8_t *>(meters.data()) +
                        meters.size() * sizeof(float));
  return image;
}

sensor_msgs::msg::PointCloud2
MockRobotNode::makePointCloud(const sensor_msgs::msg::Image &depth) const {
  // 从深度图**反投影**：每个像素 (u, v) + 它的 z 深度 → 光学系里的一个三维点。
  //   x = (u - cx) * z / fx
  //   y = (v - cy) * z / fy      （fy = fx，像素是正方形的）
  //   z = z 深度本身
  // **信息量和深度图完全一样**，只是排布从"表格"变成了"散点"。
  const double fx = cameraFocalLength();
  const double cx = kDepthWidth / 2.0;
  const double cy = kDepthHeight / 2.0;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = depth.header.stamp;
  cloud.header.frame_id = camera_optical_frame_; // 点就在光学系里，不用再转
  // **有组织**的点云：行列结构和深度图一致（真实 depth_image_proc 也是这么发的）。
  cloud.height = kDepthHeight;
  cloud.width = kDepthWidth;
  cloud.is_bigendian = false;
  // 房间是封闭的 ⇒ 每个像素都有有效深度，没有 NaN。
  cloud.is_dense = true;
  cloud.point_step = 3 * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(3);
  const char *axis_names[3] = {"x", "y", "z"};
  for (std::size_t i = 0; i < 3; ++i) {
    cloud.fields[i].name = axis_names[i];
    cloud.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[i].count = 1;
  }

  // 先在连续内存里拼好，再整体拷进 data —— 和深度图同一套做法，避免对齐假设。
  std::vector<float> xyz(static_cast<std::size_t>(kDepthWidth) * kDepthHeight * 3);
  for (int v = 0; v < kDepthHeight; ++v) {
    for (int u = 0; u < kDepthWidth; ++u) {
      const std::size_t pixel = static_cast<std::size_t>(v) * kDepthWidth + u;
      const double z = readDepthPixel(depth, pixel);
      xyz[pixel * 3 + 0] = static_cast<float>((u - cx) * z / fx);
      xyz[pixel * 3 + 1] = static_cast<float>((v - cy) * z / fx);
      xyz[pixel * 3 + 2] = static_cast<float>(z);
    }
  }
  cloud.data.assign(reinterpret_cast<const uint8_t *>(xyz.data()),
                    reinterpret_cast<const uint8_t *>(xyz.data()) +
                        xyz.size() * sizeof(float));
  return cloud;
}

} // namespace robot_sdk::sim
