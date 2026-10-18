#include "robot_sdk/detail/joint_wrap.hpp"

#include <fstream>
#include <sstream>
#include "robot_sdk_sim/mock_robot_node.hpp"
#include "robot_sdk_sim/mock_world.hpp"
#include "robot_sdk_sim/tick_action.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_action/rcl_action.h>

namespace robot_sdk::sim {
namespace {

// 三个定时器的周期：控制最密（50 Hz），状态次之（10 Hz）。
constexpr double kControlPeriodS = 0.02;
// 轮子半径（与 URDF 的 wheel_radius 对齐）：只用来把位移折算成轮子转角。
constexpr double kWheelRadius = 0.055;
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
// 以上"机器人属性"（到位容差 / 速度上限 / 车身余量）都在 RobotConfig::limits 里，由执行侧读取。
// 留在这里的都是**仿真自己的真值**（控制周期、轮半径、反馈分频、电量模型）。

// 传感器：控制回路 50 Hz → 传感器 10 Hz。
constexpr int kSensorEveryNTicks = 5;

/// 从 URDF 里取某个 fixed joint 的 origin 的 xyz（传感器挂载点）。读不到返回 NaN。
/// 与 jointSpecsFromFile 同一个动机：挂载点是机器人的属性，**不抄数字**（抄一份必然漂）。
std::array<double, 3> originFromFile(const std::string &path, const std::string &joint_name) {
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  std::array<double, 3> out{nan, nan, nan};
  std::ifstream file(path);
  if (!file) {
    return out;
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  const std::string text = buffer.str();
  const auto joint = text.find("<joint name=\"" + joint_name + "\"");
  const auto origin = joint == std::string::npos ? std::string::npos : text.find("<origin", joint);
  const auto xyz = origin == std::string::npos ? std::string::npos : text.find("xyz=\"", origin);
  if (xyz == std::string::npos) {
    return out;
  }
  std::istringstream values(text.substr(xyz + 5, text.find('"', xyz + 5) - xyz - 5));
  values >> out[0] >> out[1] >> out[2];
  return out;
}
// 底盘命令：超过这么久没收到新的 cmd_vel 就当停车。
// 真底盘也这样 —— 命令丢了不该继续冲。
constexpr double kCmdVelTimeoutS = 0.5;
// 激光：一圈 1° 分辨率，量程照常见 2D 雷达填。
constexpr int kScanRays = 360;
constexpr double kScanRangeMin = 0.05;
constexpr double kScanRangeMax = 10.0;

// 房间（模拟世界）与 rayToBox 都在 mock_world.hpp —— 抽出去是为了能单测纯函数。

// 深度相机：尺寸小就够 —— 它是让客户"有东西可读"，不是图像质量展示。
constexpr int kDepthWidth = 64;
constexpr int kDepthHeight = 48;
constexpr double kCameraHfovRad = M_PI / 3.0; // 水平视场 60°

/// 深度相机的焦距（像素）—— 由"水平视场 60°"推出来，而不是随手写个数。
/// 像素是正方形的，所以 fx = fy。
double cameraFocalLength() {
  return (kDepthWidth / 2.0) / std::tan(kCameraHfovRad / 2.0);
}

/// 读深度图的第 `index` 个像素（encoding = "32FC1"，每像素一个 float）。
/// 用 memcpy 而不是把 data 强转成 float*：后者依赖"缓冲区满足 float 对齐"这个标准里不保证的假设。
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

  // 关节：优先用配置里的；没配就给一套默认（参考平台是 6 轴臂）。
  joint_names_ = config_.motion.joint_names;
  if (joint_names_.empty()) {
    joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                    "wrist_joint", "wrist_pitch_joint", "wrist_roll_joint"};
  }
  joint_positions_.assign(joint_names_.size(), 0.0);
  if (!config_.motion.gripper_joint.empty()) {
    ensureJoint(config_.motion.gripper_joint);
  }

  // 关节的属性（连续不连续 / 限位 / 速度上限）来自**机器人自己的 URDF**。
  // 这里不查、也不存在配置里 —— 那是"执行侧"才知道的事，SDK 只发意图。
  // 没给描述就退回老行为：不折角、不查限位（也就是少了这两项判断）。
  const std::string urdf = declare_parameter<std::string>("robot_description_file", "");
  if (!urdf.empty()) {
    joint_specs_ = jointSpecsFromFile(urdf, joint_names_);
    if (joint_specs_.empty()) {
      RCLCPP_WARN(get_logger(), "读不到关节描述 %s —— 限位不查、也不折角",
                  urdf.c_str());
    }
  }

  // 话题名一律用**相对名**，由 rcl 按节点命名空间展开。
  status_pub_ = create_publisher<RobotStatus>(config_.status.status_topic, 10);
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      config_.status.joint_states_topic, 10);

  // 传感器用**与 SDK 侧同一个** QoS 定义 —— 直接问 `SubscriptionQos` 要，别再自己拼一遍：
  // 两边不一致的话订阅会**静默收不到**数据，而"同一个知识两处各存一份"迟早漂移。
  auto sensor_qos = config_.qos.sensor.toQos();
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

  // 3D 模型的数据源。此前**没人发它** —— 于是 RViz 里看得见网格与传感器、
  // 看不见机器人本体。transient_local 与 robot_state_publisher 同一约定：
  // 迟到的订阅者（后开的 RViz）也能拿到；⚠️ 订阅端也必须请求 Transient Local。
  description_pub_ = create_publisher<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local());
  if (!urdf.empty()) {
    std::ifstream file(urdf);
    std::stringstream buffer;
    buffer << file.rdbuf();
    if (file) {
      std_msgs::msg::String description;
      description.data = buffer.str();
      description_pub_->publish(description);
    } else {
      RCLCPP_WARN(get_logger(), "读不到 URDF 文件 %s —— RViz 里看不到机器人本体",
                  urdf.c_str());
    }
  }

  // 只发**世界位姿** map→odom→base_footprint（URDF 不可能知道"机器人在哪"）。传感器挂载点
  // 不由本节点发 —— 它们是 URDF 的 fixed joint，归 robot_state_publisher 发到 /tf_static；
  // 自己再发一份就是"同一个帧两个父节点" ⇒ TF_REPEATED_DATA。
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 唯一要用到挂载点的是 raycast 的**起点**（激光 x/z、相机 x/z）—— 从 URDF 读，不抄数字。
  if (!urdf.empty()) {
    const auto laser = originFromFile(urdf, "laser_joint");
    const auto camera = originFromFile(urdf, "camera_joint");
    if (!std::isnan(laser[0])) laser_mount_x_ = laser[0];
    if (!std::isnan(laser[2])) laser_mount_z_ = laser[2];
    if (!std::isnan(camera[0])) camera_mount_x_ = camera[0];
    if (!std::isnan(camera[2])) camera_mount_z_ = camera[2];
    if (std::isnan(laser[0]) || std::isnan(camera[0])) {
      RCLCPP_WARN(get_logger(), "读不到传感器挂载点 %s —— 用编译期默认值", urdf.c_str());
    }
  }

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
          // 与 /joint_states 一致：轮子也报（否则两条路径的关节表不一样）
          response->joint_states.name.push_back("left_wheel_joint");
          response->joint_states.name.push_back("right_wheel_joint");
          response->joint_states.position.push_back(wheel_angle_);
          response->joint_states.position.push_back(wheel_angle_);
          response->joint_states.velocity.assign(response->joint_states.name.size(), 0.0);
          response->joint_states.effort.assign(response->joint_states.name.size(), 0.0);
        }
      },
      rclcpp::ServicesQoS(), service_group_);

  // 唯一性：flock 锁文件。范围是「本机 + 本域」—— 同域就是同一个 DDS 图，与哪个用户无关。
  // 不同域可并存；同域要跑多个：--ros-args -p unique:=false
  if (declare_parameter<bool>("unique", true)) {
    const char *dir = std::getenv("XDG_RUNTIME_DIR");  // 没有则退回 /tmp
    const char *domain = std::getenv("ROS_DOMAIN_ID");
    const std::string lock_path =
        std::string(dir ? dir : "/tmp") + "/mock_robot_" + (domain ? domain : "0") + ".lock";
    // flock 只读也能加锁；0666 是让不同用户也能打开同一个锁文件（我们从不写它）。
    lock_fd_ = ::open(lock_path.c_str(), O_RDONLY | O_CREAT, 0666);
    if (lock_fd_ < 0) {
      // fail-closed：打不开锁就拒绝启动 —— 否则会悄悄失去唯一性保护。
      RCLCPP_FATAL(get_logger(), "打不开锁文件 %s：%s —— 拒绝启动", lock_path.c_str(),
                   std::strerror(errno));
      rclcpp::shutdown();
      std::exit(1);
    } else if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      RCLCPP_FATAL(get_logger(), "锁 %s 已被占用，拒绝启动（要并存加 -p unique:=false）",
                   lock_path.c_str());
      rclcpp::shutdown();
      std::exit(1);
    }
  }

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

  // 底盘命令入口。话题名是 **mock 自己的参数**（不属于 SDK 契约：客户从不发 cmd_vel），
  // 这样换 Nav2 配置时不用改代码。
  // ⚠️ 收 **TwistStamped**：对齐 Jazzy 的 diff_drive_controller（它只收 stamped），
  //    这样 Nav2 发的东西**同一个类型**能同时喂仿真底盘和真/仿真底盘控制器，不必再插转换节点。
  const std::string cmd_vel_topic =
      declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  cmd_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions cmd_options;
  cmd_options.callback_group = cmd_group_;
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic, rclcpp::QoS(10),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_linear_ = msg->twist.linear.x;
        cmd_angular_ = msg->twist.angular.z;
        cmd_stamp_ = std::chrono::steady_clock::now();
        cmd_received_ = true;
      },
      cmd_options);

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
  // 参数不命名 = 一眼看出它是**凭证**不是数据；它不是"自己加锁"，所以这里绝不能再 lock。
  // 后续加导航/运动状态时**只在这里加分支**，状态话题与服务会自动同步。
  RobotStatus status;
  status.header.stamp = now();
  status.header.frame_id = config_.navigation.base_frame;

  // 状态**位**：两个运动通道各占一位（能同时成立）。旧版用 else-if 串成单值 ⇒ 同时动时丢信息。
  // ⚠️ 底盘那一位要覆盖**两条**路径：导航动作，以及 cmd_vel 直接控速（Nav2 走的也是后者）。
  const bool base_moving =
      nav_.slot.running() ||
      (cmd_received_ && std::chrono::steady_clock::now() - cmd_stamp_ <
                            std::chrono::duration<double>(kCmdVelTimeoutS));
  status.state_bits = 0;
  if (base_moving) {
    status.state_bits |= RobotStatus::STATE_BASE_MOVING;
  }
  if (motion_.slot.running()) {
    status.state_bits |= RobotStatus::STATE_ARM_MOVING;
  }
  if (charging_) {
    status.state_bits |= RobotStatus::STATE_CHARGING;
  }
  if (battery_ < kLowBatteryThreshold) {
    status.state_bits |= RobotStatus::STATE_BATTERY_LOW;
  }

  status.battery_percentage = static_cast<float>(battery_);
  status.battery_voltage = static_cast<float>(
      kBatteryNominalVoltage + kBatteryVoltagePerPercent * battery_);
  status.uptime_seconds = uptime_s_;

  // 模拟机器人没有故障、也没有急停按钮 —— 显式写零，而不是靠消息默认值。
  // （急停没有独立字段了：它就是 state_bits 里那一位；没置位即"没急停"。）
  status.error_code = 0;
  status.error_message = "";
  return status;
}

void MockRobotNode::navTick() {
  // 骨架负责加锁/空转/取消/终态回调/反馈发布；这里只写"这一拍推进了什么"。
  tickAction<NavigateToPose>(
      mutex_, nav_.slot,
      [this](std::lock_guard<std::mutex> &lock) -> TickOutput<NavigateToPose> {
        const double remaining = distance(pose_, nav_.target);
        // ⚠️ 必须**同时**看朝向：只判 xy 的话，"原地转身"（xy 没变）会被当成已到达 ⇒ 一步跳到位。
        const double yaw_error = std::abs(normalizeAngle(nav_.target.yaw - pose_.yaw));
        if (remaining <= config_.limits.goal_tolerance_m &&
            yaw_error <= config_.limits.yaw_tolerance_rad) {
          pose_ = nav_.target; // 对齐，免得留下毫米级残差
          auto result = std::make_shared<NavigateToPose::Result>();
          result->success = true;
          result->final_distance_error = static_cast<float>(remaining);
          result->message = "reached target";
          return {result, nullptr}; // 结束（骨架会收尾）
        }

        // avoid_obstacles：朝目标方向看一眼，够不够走到。挡着就停下 ——
        // 这个模拟器**不会绕路**（绕路是规划器的活），它只保证不撞。
        if (nav_.avoid_obstacles) {
          const double wall = rayToBox(pose_.x, pose_.y, laser_mount_z_,
                                       (nav_.target.x - pose_.x) / remaining,
                                       (nav_.target.y - pose_.y) / remaining, 0.0);
          if (wall < remaining + config_.limits.body_clearance_m) {
            auto result = std::make_shared<NavigateToPose::Result>();
            result->success = false;
            result->final_distance_error = static_cast<float>(remaining);
            result->message = "前方被挡，不走了";
            return {result, nullptr};
          }
        }

        // 朝目标走一步，并把朝向转向目标（模拟一个会转头的前进底盘）。
        // ⚠️ remaining 可能为 0（原地转身时位置没动）：拿它做除数会 0/0=NaN，把 pose 毒死，
        // 之后每一帧 TF 全是 NaN（RViz 直接丢弃 ⇒ 模型/轮子全红）。所以两种推进都要判。
        const double want =
            remaining > 1e-9
                ? std::atan2(nav_.target.y - pose_.y, nav_.target.x - pose_.x)
                : nav_.target.yaw; // 位置已到：转成目标朝向
        const double max_turn = config_.limits.max_yaw_rate * kControlPeriodS;
        pose_.yaw = normalizeAngle(
            pose_.yaw + std::clamp(normalizeAngle(want - pose_.yaw), -max_turn, max_turn));
        if (remaining > 1e-9) { // 位置还没到才平移
          const double step = std::min(nav_.max_speed * kControlPeriodS, remaining);
          pose_.x += step * (nav_.target.x - pose_.x) / remaining;
          pose_.y += step * (nav_.target.y - pose_.y) / remaining;
        }

        if (++nav_.feedback_divider >= kFeedbackEveryNTicks) {
          nav_.feedback_divider = 0;
          return {nullptr,
                  std::make_shared<NavigateToPose::Feedback>(makeNavFeedback(lock))};
        }
        return {}; // 这一拍只是推进了，没有要报的
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
    busy = nav_.slot.busy();
    if (!busy) {
      // reserve() 只置"占位"相位，**不置 kRunning** —— 句柄要等 accepted 回调才到，
      // 而 kRunning 一旦成立，navTick 就会拿句柄去回调框架，那个窗口里它还是空的。
      // 它顺手清掉残留的取消请求（与取消回调存在竞态，见 学习笔记.md §7.8）。
      nav_.slot.reserve();
      nav_.feedback_divider = 0;
      nav_.target = toPose2D(goal->target_pose);
      // 本次请求 vs 能力上限：客户没给就取上限；给了也不能超过上限 ——
      // 约束是**执行侧**的责任（SDK 只转发意图）。超了要说一声，免得客户以为按 5 m/s 在跑。
      const double requested = goal->max_speed > 0.0f ? goal->max_speed
                                                     : config_.limits.max_speed;
      if (requested > config_.limits.max_speed) {
        RCLCPP_WARN(get_logger(), "请求速度 %.2f 超过能力上限 %.2f，按上限执行", requested,
                    config_.limits.max_speed);
      }
      nav_.max_speed = std::min(requested, config_.limits.max_speed);
      nav_.avoid_obstacles = goal->avoid_obstacles;
    }
  }

  if (busy) {
    RCLCPP_WARN(get_logger(), "拒绝导航目标：已经在导航中");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal->avoid_obstacles) {
    RCLCPP_WARN(get_logger(), "avoid_obstacles=false：路上撞墙也不管");
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
  nav_.slot.cancel_requested = true;
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
    nav_.slot.begin(goal_handle);
    target = nav_.target;
    speed = nav_.max_speed;
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
      static_cast<float>(distance(pose_, nav_.target));
  feedback.estimated_time_remaining =
      nav_.max_speed > 0.0
          ? static_cast<float>(feedback.distance_remaining / nav_.max_speed)
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
    worst = std::max(worst, std::abs(motion_.target[i] - joint_positions_[i]));
  }
  // 完成度 = 从"起始最大误差"缩到"当前最大误差"的比例；起始就是 0 视为已完成。
  feedback.completion_percentage =
      motion_.initial_error > 0.0
          ? static_cast<float>(100.0 * (1.0 - worst / motion_.initial_error))
          : 100.0f;
  return feedback;
}

void MockRobotNode::motionTick() {
  // 同 navTick：骨架一样，只有中间这段"推进"不同。
  tickAction<MoveToJointPositions>(
      mutex_, motion_.slot,
      [this](std::lock_guard<std::mutex> &lock) -> TickOutput<MoveToJointPositions> {
        // 每个关节各朝目标走一步，速度走**梯形**曲线：上限是 max_velocity；靠近目标时
        // 按"现在开始减速、正好停住"降速，不会冲过头；速度每拍最多变 max_accel * 周期，
        // 所以起步和收尾是渐进的。
        const double dt = kControlPeriodS;
        const double accel = motion_.max_acceleration;
        const double dv = accel > 0.0 ? accel * dt : std::numeric_limits<double>::infinity();
        double worst = 0.0;
        for (std::size_t i = 0; i < joint_positions_.size(); ++i) {
          const double error = motion_.target[i] - joint_positions_[i];
          const double distance = std::abs(error);
          // "走完这一拍就到位"和"减速正好停住"取小的，再压到速度上限。
          double wanted = distance / dt;
          if (accel > 0.0) {
            wanted = std::min(wanted, std::sqrt(2.0 * accel * distance));
          }
          wanted = std::min(wanted, motion_.max_velocity);

          double &velocity = motion_.velocities[i];
          velocity = std::clamp(wanted, velocity - dv, velocity + dv);

          const double step = velocity * dt;
          if (distance <= step) {
            joint_positions_[i] = motion_.target[i];
            velocity = 0.0; // 到位就停住，别让速度留到下一个目标
          } else {
            joint_positions_[i] += std::copysign(step, error);
          }
          worst = std::max(worst, std::abs(motion_.target[i] - joint_positions_[i]));
        }

        if (worst <= config_.limits.joint_tolerance) {
          joint_positions_ = motion_.target; // 对齐，免得留下残差
          auto result = std::make_shared<MoveToJointPositions::Result>();
          result->success = true;
          result->final_position_error = static_cast<float>(worst);
          result->message = "reached target";
          return {result, nullptr};
        }
        // 反馈不必跟控制同频（同导航）：50 Hz 控制、10 Hz 反馈。
        if (++motion_.feedback_divider >= kFeedbackEveryNTicks) {
          motion_.feedback_divider = 0;
          return {nullptr,
                  std::make_shared<MoveToJointPositions::Feedback>(makeMotionFeedback(lock))};
        }
        return {};
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
  std::string out_of_range{}; // 非空 = 有关节超限，内容是那句人话
  {
    // 锁内只碰数据：先在**局部**把目标拼好，全部合法才提交（不留半成品状态）。
    std::lock_guard<std::mutex> lock(mutex_);
    busy = motion_.slot.busy();
    if (!busy) {
      std::vector<double> target = joint_positions_; // 目标没点名的关节保持原位
      for (std::size_t k = 0; k < goal->target.name.size(); ++k) {
        const auto it =
            std::find(joint_names_.begin(), joint_names_.end(), goal->target.name[k]);
        if (it == joint_names_.end()) {
          unknown = true;
          break;
        }
        const std::size_t index = static_cast<std::size_t>(it - joint_names_.begin());
        double wanted = goal->target.position[k];
        // 关节描述是可选的：没有它（或它里面没这个关节）就不判限位、不折角。
        // 找到之后才取引用 —— 迭代器等于 end() 时解引用是 UB，不能无条件取。
        if (const auto spec_it = joint_specs_.find(goal->target.name[k]);
            spec_it != joint_specs_.end()) {
          const JointSpec &spec = spec_it->second;
          // 限位：**执行侧**才判"能不能到"。超限的目标当场拒，不转过去 ——
          // 真机上那就是撞机械限位。
          if (spec.limited() && (wanted < spec.lower || wanted > spec.upper)) {
            out_of_range = goal->target.name[k] + " 的目标 " + std::to_string(wanted) +
                           " 超出限位 [" + std::to_string(spec.lower) + ", " +
                           std::to_string(spec.upper) + "]";
            break;
          }
          // 连续关节：走最短的一边，否则 0.3 → 7.0 会老实转 6.7 rad（≈384°）。
          if (spec.continuous) {
            wanted = joint_positions_[index] + shortestAngleDelta(joint_positions_[index], wanted);
          }
        }
        target[index] = wanted;
      }
      if (!unknown && out_of_range.empty()) {
        motion_.target = std::move(target);
        motion_.max_velocity =
            goal->max_velocity > 0.0f ? goal->max_velocity
                                      : config_.limits.default_joint_velocity;
        // <= 0 表示"由后端决定" —— 对模拟器就是不限加速度。
        motion_.max_acceleration = goal->max_acceleration;
        // reserve() 只置"占位"相位（理由同 handleNavGoal），并清掉残留的取消请求。
        motion_.slot.reserve();
        motion_.velocities.assign(joint_positions_.size(), 0.0); // 从静止开始
        motion_.feedback_divider = 0;
        // 起始最大误差 —— completion_percentage 的分母。
        motion_.initial_error = 0.0;
        for (std::size_t i = 0; i < motion_.target.size(); ++i) {
          motion_.initial_error = std::max(
              motion_.initial_error, std::abs(motion_.target[i] - joint_positions_[i]));
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
  if (!out_of_range.empty()) {
    RCLCPP_WARN(get_logger(), "拒绝运动目标：%s", out_of_range.c_str());
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
  motion_.slot.cancel_requested = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MockRobotNode::handleMotionAccepted(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  motion_.slot.begin(goal_handle); // 句柄与相位同时改变（理由同 handleNavAccepted）
}

void MockRobotNode::cmdVelTick() {
  std::lock_guard<std::mutex> lock(mutex_);

  const bool stale =
      !cmd_received_ ||
      (std::chrono::steady_clock::now() - cmd_stamp_) >
          std::chrono::duration<double>(kCmdVelTimeoutS);
  const double linear = stale ? 0.0 : cmd_linear_;
  const double angular = stale ? 0.0 : cmd_angular_;
  if (linear == 0.0 && angular == 0.0) {
    return;
  }

  // 差速底盘：先转向，再沿**转完之后**的车头方向前进一个周期的行程。
  pose_.yaw = normalizeAngle(pose_.yaw + angular * kControlPeriodS);
  pose_.x += linear * kControlPeriodS * std::cos(pose_.yaw);
  pose_.y += linear * kControlPeriodS * std::sin(pose_.yaw);
}

void MockRobotNode::controlTick() {
  // cmdVelTick 和 navTick 都能改位姿 —— 接 Nav2 时用前者，用本节点的导航服务端时用后者。
  cmdVelTick();
  navTick();
  motionTick();

  // 一拍**只取一次时间**：同一拍产生的关节 / TF / 传感器必须共享时间戳。
  // 各打各的 `now()` 的话，下游按时间对齐时会发现"同一拍的数据时间对不上"，
  // Nav2 的 message filter 就会把激光丢掉。
  const rclcpp::Time stamp = now();
  publishJoints(stamp);
  publishFrames(stamp);
  publishSensors(stamp); // 内部降频到 10 Hz
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

void MockRobotNode::publishFrames(const rclcpp::Time &stamp) {
  Pose2D pose;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose = pose_;
  }
  // 锁外做 I/O —— sendTransform 会走 DDS，不该占着锁（同 publishJoints）。
  // 这里是模拟机器人的"真值"：它假装 odom 与 global_frame 重合，
  // 于是把位姿直接发成 odom -> base 的变换。
  tf_broadcaster_->sendTransform(makeTransform(
      stamp, config_.navigation.global_frame, "odom", 0.0, 0.0, 0.0, 0.0));
  tf_broadcaster_->sendTransform(
      makeTransform(stamp, "odom", config_.navigation.base_frame, pose.x, pose.y, 0.0,
                    pose.yaw));
}

void MockRobotNode::publishJoints(const rclcpp::Time &stamp) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.name = joint_names_;
    msg.position = joint_positions_;
    // 轮子也要上报：`robot_state_publisher` 只对 /joint_states 里出现过的**可动**关节发 TF，
    // 而底盘位置是这个节点自己广播的 —— 不补这两项，RViz 就找不到 left/right_wheel_link。
    // 转角由位姿增量折算（差速底盘看不出左右差，够 RViz 用）。
    const double travelled =
        std::hypot(pose_.x - last_wheel_pose_.x, pose_.y - last_wheel_pose_.y);
    wheel_angle_ += travelled / kWheelRadius;
    last_wheel_pose_ = pose_;
    msg.name.push_back("left_wheel_joint");
    msg.name.push_back("right_wheel_joint");
    msg.position.push_back(wheel_angle_);
    msg.position.push_back(wheel_angle_);
    msg.velocity.assign(msg.name.size(), 0.0);
    msg.effort.assign(msg.name.size(), 0.0);
  }
  joint_pub_->publish(msg);
}

void MockRobotNode::publishSensors(const rclcpp::Time &stamp) {
  sensor_msgs::msg::LaserScan scan;
  sensor_msgs::msg::Imu imu;
  sensor_msgs::msg::CameraInfo camera_info;
  sensor_msgs::msg::Image depth;

  {
    // ── 锁内：降频 + 组装（不碰发布）──
    std::lock_guard<std::mutex> lock(mutex_);

    // 降频判断其实可以放锁外（sensor_divider_ 只有控制回路这条车道碰），但仍放锁内，
    // 为的是守住"mutex_ 下面那块成员全部在锁内访问、没有例外"。挪出去只省一次纳秒级的
    // 无争用 lock，却引入"这个成员不受保护"的例外 ⇒ 日后别的车道顺手读就是静默数据竞争。
    if (++sensor_divider_ < kSensorEveryNTicks) {
      return; // 还没到下一拍
    }
    sensor_divider_ = 0;

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
  const double laser_x = pose_.x + laser_mount_x_ * std::cos(pose_.yaw);
  const double laser_y = pose_.y + laser_mount_x_ * std::sin(pose_.yaw);

  scan.ranges.reserve(kScanRays);
  for (int i = 0; i < kScanRays; ++i) {
    // angle 是**激光坐标系**里的角 —— 机器人转了，世界角就跟着转。
    const double angle = scan.angle_min + i * scan.angle_increment;
    const double world_angle = pose_.yaw + angle;
    // 激光射线是**水平的**（dz = 0），所以地面/天花板那两项永远命中不了。
    // 与相机共用同一个 `rayToBox` —— 一份房间模型，不会改了一个忘另一个。
    const double range = rayToBox(laser_x, laser_y, laser_mount_z_,
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

  // 相机安装点：从 URDF 读（读不到才用编译期默认值），随机器人朝向一起转。
  const double cam_x = pose_.x + camera_mount_x_ * std::cos(pose_.yaw);
  const double cam_y = pose_.y + camera_mount_x_ * std::sin(pose_.yaw);
  const double cam_z = camera_mount_z_;
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

      // 光学 (x 右, y 下, z 前) → body (x 前, y 左, z 上)：body = (z, -x, -y)。
      // 这一步对应上面那条 RPY(-90°, 0, -90°) 的静态 TF —— 两者必须配套，
      // 否则深度图和 TF 各说各话（而且都不报错）。
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
  // 从深度图**反投影**：x = (u - cx) * z / fx，y = (v - cy) * z / fx，z 就是深度本身。
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
