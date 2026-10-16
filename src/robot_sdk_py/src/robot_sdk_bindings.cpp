// Robot SDK 的 Python 绑定（pybind11）。**只有一个实现**：调的就是 C++ 那份核心。
// ⚠️ 边界约定：C++ 侧刻意不用异常，而**语言边界正是最该抛的地方** ⇒ 成功投影成 dict、
//    失败抛 SdkException（Python 里的 RobotSdkError）。GIL 的两条规矩见包里的 README。
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "robot_sdk/detail/action_util.hpp"
#include "robot_sdk/robot_sdk.hpp"

namespace py = pybind11;
using namespace robot_sdk;

using RobotStatus = robot_sdk_interfaces::msg::RobotStatus;

namespace {

/// 放开 GIL 跑一段**纯 C++**（返回值是 C++ 对象；回到 Python 前 GIL 已拿回）。
template <typename Fn>
auto withoutGil(Fn&& fn) {
  py::gil_scoped_release release;
  return fn();
}

py::dict toDict(const Pose2D& pose) {
  py::dict out;
  out["x"] = pose.x;
  out["y"] = pose.y;
  out["yaw"] = pose.yaw;
  return out;
}

py::dict toDict(const MotionResult& result) {
  py::dict out;
  out["success"] = result.success;
  out["final_position_error"] = result.final_position_error;
  out["message"] = result.message;
  return out;
}

py::dict toDict(const PoseResult& result) {
  py::dict out;
  out["success"] = result.success;
  out["final_position_error"] = result.final_position_error;
  out["final_orientation_error"] = result.final_orientation_error;
  out["message"] = result.message;
  return out;
}

py::dict toDict(const NavigationResult& result) {
  py::dict out;
  out["success"] = result.success;
  out["final_distance_error"] = result.final_distance_error;
  out["message"] = result.message;
  return out;
}

py::dict toDict(const RobotStatus& status) {
  py::dict out;
  out["state_bits"] = status.state_bits;
  out["battery_percentage"] = status.battery_percentage;
  out["battery_voltage"] = status.battery_voltage;
  out["error_code"] = status.error_code;
  out["uptime_seconds"] = status.uptime_seconds;
  // 判据由 SDK 一处给出（含 fail-closed）—— 调用方不要自己位运算。
  out["allows_motion"] = motionAllowed(status);
  out["allows_navigation"] = navigationAllowed(status);
  return out;
}

py::dict toDict(const sensor_msgs::msg::JointState& state) {
  py::dict out;
  out["name"] = state.name;
  out["position"] = state.position;
  out["velocity"] = state.velocity;
  out["effort"] = state.effort;
  return out;
}

py::dict toDict(const sensor_msgs::msg::LaserScan& scan) {
  py::dict out;
  out["frame_id"] = scan.header.frame_id;
  out["angle_min"] = scan.angle_min;
  out["angle_max"] = scan.angle_max;
  out["angle_increment"] = scan.angle_increment;
  out["range_min"] = scan.range_min;
  out["range_max"] = scan.range_max;
  out["ranges"] = scan.ranges;
  // 便捷量：客户十有八九要的是"多少条射线"，不必自己去数 ranges。
  out["beam_count"] = scan.ranges.size();
  return out;
}

py::dict toDict(const sensor_msgs::msg::Imu& imu) {
  py::dict out;
  out["frame_id"] = imu.header.frame_id;
  out["angular_velocity"] = py::make_tuple(imu.angular_velocity.x, imu.angular_velocity.y,
                                          imu.angular_velocity.z);
  out["linear_acceleration"] = py::make_tuple(imu.linear_acceleration.x,
                                              imu.linear_acceleration.y,
                                              imu.linear_acceleration.z);
  out["orientation"] = py::make_tuple(imu.orientation.x, imu.orientation.y,
                                      imu.orientation.z, imu.orientation.w);
  return out;
}

py::dict toDict(const sensor_msgs::msg::Image& image) {
  py::dict out;
  out["frame_id"] = image.header.frame_id;
  out["width"] = image.width;
  out["height"] = image.height;
  out["encoding"] = image.encoding;
  out["step"] = image.step;
  // ⚠️ **不**把 data 拷进 bytes：深度图/点云动辄几百 KB~数 MB，
  //    每次读都拷一遍会让"最新一帧"这个语义变得很贵。要像素自己 bytes(msg) 取。
  return out;
}

py::dict toDict(const sensor_msgs::msg::PointCloud2& cloud) {
  py::dict out;
  out["frame_id"] = cloud.header.frame_id;
  out["width"] = cloud.width;
  out["height"] = cloud.height;
  out["point_step"] = cloud.point_step;
  out["is_dense"] = cloud.is_dense;
  return out;
}

py::dict toDict(const sensor_msgs::msg::CameraInfo& info) {
  py::dict out;
  out["frame_id"] = info.header.frame_id;
  out["width"] = info.width;
  out["height"] = info.height;
  out["k"] = info.k;
  return out;
}

// ── 反馈的投影（`on_feedback` 交给 Python 的必须是 dict，ROS 消息类型没绑）──

/// PoseStamped 用 (x, y, z, qx, qy, qz, qw) 七元组表达 —— 和 PoseGoal.target 同一形状。
py::tuple poseToTuple(const geometry_msgs::msg::PoseStamped& stamped) {
  return py::make_tuple(stamped.pose.position.x, stamped.pose.position.y,
                        stamped.pose.position.z, stamped.pose.orientation.x,
                        stamped.pose.orientation.y, stamped.pose.orientation.z,
                        stamped.pose.orientation.w);
}

py::dict toDict(const robot_sdk_interfaces::action::MoveToJointPositions::Feedback& feedback) {
  py::dict out;
  out["current"] = toDict(feedback.current);
  out["completion_percentage"] = feedback.completion_percentage;
  return out;
}

py::dict toDict(const robot_sdk_interfaces::action::MoveToPose::Feedback& feedback) {
  py::dict out;
  out["current_pose"] = poseToTuple(feedback.current_pose);
  out["completion_percentage"] = feedback.completion_percentage;
  return out;
}

py::dict toDict(const robot_sdk_interfaces::action::NavigateToPose::Feedback& feedback) {
  py::dict out;
  out["current_pose"] = poseToTuple(feedback.current_pose);
  out["distance_remaining"] = feedback.distance_remaining;
  out["estimated_time_remaining"] = feedback.estimated_time_remaining;
  out["current_state"] = feedback.current_state;
  return out;
}

// ── Result / optional 的边界翻译（**必须放在所有 toDict 之后**，下面要用到它们）──

/// 把各种"值"投影成 dict。做成仿函数而不是直接传 `toDict`：`toDict` 是**重载**的，
/// 传名字没法推导出要哪一个（编译期就失败）。
struct Projector {
  py::dict operator()(const Pose2D& value) const { return toDict(value); }
  py::dict operator()(const MotionResult& value) const { return toDict(value); }
  py::dict operator()(const PoseResult& value) const { return toDict(value); }
  py::dict operator()(const NavigationResult& value) const { return toDict(value); }
  py::dict operator()(const RobotStatus& value) const { return toDict(value); }

  // 感知/状态的等待返回的是 shared_ptr<const Msg> —— 解引用后再投影。
  template <typename Msg>
  py::dict operator()(const std::shared_ptr<const Msg>& value) const {
    return toDict(*value);
  }
};

/// Result 的边界翻译：失败抛异常（Python 侧是 RobotSdkError），成功投影成 dict。
template <typename T>
py::dict unwrap(const Result<T>& result, Projector project = {}) {
  if (!result.ok()) {
    throw SdkException(result.error());
  }
  return project(result.value());
}

/// optional 的边界翻译：有值 ⇒ dict；没有 ⇒ None（**不抛**）。
/// 非阻塞的"读最新一帧"问的是"现在有吗"，没有不是错误。
template <typename Ptr>
py::object projectOptional(const std::optional<Ptr>& value, Projector project = {}) {
  if (!value || !*value) {
    return py::none();
  }
  return project(*value);
}

/// 把 Python 回调包成"**可以在任意线程自由拷贝**"的 C++ 回调。
/// ⚠️ C++ 侧会在没有 GIL 的线程里拷这个 std::function，所以壳里**不能**直接放 py::object
///    （拷贝=inc_ref、析构=decref，都会当场 core dump）；用 shared_ptr 包住，只在带 GIL 时碰计数。
struct GuardedCallback {
  std::shared_ptr<py::object> held;

  explicit GuardedCallback(py::function callback)
      : held(new py::object(std::move(callback)), [](py::object* object) {
          py::gil_scoped_acquire acquire;
          delete object;
        }) {}

  /// 动作反馈：C++ 侧给的是 shared_ptr<const Feedback>。
  template <typename Feedback>
  std::function<void(std::shared_ptr<const Feedback>)> forHandle() const {
    auto capture = held;
    return [capture](std::shared_ptr<const Feedback> msg) {
      if (!msg) {
        return;
      }
      py::gil_scoped_acquire acquire;
      (*capture)(toDict(*msg));
    };
  }

  /// 状态推送：C++ 侧给的是 const 引用。
  std::function<void(const RobotStatus&)> forStatus() const {
    auto capture = held;
    return [capture](const RobotStatus& status) {
      py::gil_scoped_acquire acquire;
      (*capture)(toDict(status));
    };
  }
};

/// ActionHandle 的共有部分：两种 action 各绑一次，逻辑一样。
template <typename Action>
void bindHandle(py::module_& m, const char* name, const char* doc) {
  using Handle = ActionHandle<Action>;
  py::class_<Handle, std::shared_ptr<Handle>>(m, name, doc)
      .def("valid", &Handle::valid, "目标是否已经发出去过。")
      .def("cancel", &Handle::cancel,
           "请求取消并等取消响应；返回服务端是否受理。要确认真的停下来，再调 wait() 看终态是否 CANCELED。")
      .def(
          "wait_for_acceptance",
          [](Handle& handle, double timeout_s) {
            return withoutGil([&] {
              return handle.waitForAcceptance(std::chrono::duration<double>(timeout_s));
            });
          },
          py::arg("timeout_s"))
      .def(
          "wait",
          [](Handle& handle, double timeout_s) {
            using Wrapped = typename Handle::WrappedResult;
            const Result<Wrapped> wrapped = withoutGil([&] {
              return handle.wait(std::chrono::duration<double>(timeout_s));
            });
            if (!wrapped.ok()) {
              throw SdkException(wrapped.error());
            }
            // ⚠️ 投影里的代码是**通信层**的终态；动作自己的 success 是另一层，别混。
            const Wrapped& value = wrapped.value();
            py::dict out;
            out["code"] = resultCodeToString(value.code);
            out["result_available"] = static_cast<bool>(value.result);
            return out;
          },
          py::arg("timeout_s"))
      .def(
          "on_feedback",
          [](Handle& handle, py::function callback) {
            handle.onFeedback(
                GuardedCallback(std::move(callback))
                    .template forHandle<typename Action::Feedback>());
          },
          py::arg("callback"));
}

} // namespace

PYBIND11_MODULE(_robot_sdk, m) {
  m.doc() = "Robot SDK 的 Python 前端（实现全在 C++ 侧）";

  // ── 异常：所有失败都在这里落地，Python 侧只需 except RobotSdkError ──
  py::register_exception<SdkException>(m, "RobotSdkError");

  py::enum_<ErrorCode>(m, "ErrorCode")
      .value("OK", ErrorCode::kOk)
      .value("TIMEOUT", ErrorCode::kTimeout)
      .value("CANCELLED", ErrorCode::kCancelled)
      .value("UNAVAILABLE", ErrorCode::kUnavailable)
      .value("NOT_INITIALIZED", ErrorCode::kNotInitialized)
      .value("INVALID_ARGUMENT", ErrorCode::kInvalidArgument)
      .value("OUT_OF_RANGE", ErrorCode::kOutOfRange)
      .value("NOT_SUPPORTED", ErrorCode::kNotSupported)
      .value("INTERNAL", ErrorCode::kInternal);

  py::class_<Pose2D>(m, "Pose2D", "平面位姿（x, y, yaw）。")
      .def(py::init<>())
      .def(py::init([](double x, double y, double yaw) { return Pose2D{x, y, yaw}; }),
           py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("yaw") = 0.0)
      .def_readwrite("x", &Pose2D::x)
      .def_readwrite("y", &Pose2D::y)
      .def_readwrite("yaw", &Pose2D::yaw)
      .def("__repr__", [](const Pose2D& p) {
        return "Pose2D(x=" + std::to_string(p.x) + ", y=" + std::to_string(p.y) +
               ", yaw=" + std::to_string(p.yaw) + ")";
      });

  // ── 配置：字段基本一一对应，但用的是**当前**的字段名 ──
  py::class_<NavigationConfig>(m, "NavigationConfig")
      .def(py::init<>())
      .def_readwrite("action_name", &NavigationConfig::action_name)
      .def_readwrite("global_frame", &NavigationConfig::global_frame)
      .def_readwrite("base_frame", &NavigationConfig::base_frame)
      .def_readwrite("default_timeout_s", &NavigationConfig::default_timeout_s)
      .def_readwrite("server_wait_timeout_s", &NavigationConfig::server_wait_timeout_s)
      .def_readwrite("arrival_tolerance_m", &NavigationConfig::arrival_tolerance_m)
      .def_readwrite("arrival_yaw_tolerance_rad",
                     &NavigationConfig::arrival_yaw_tolerance_rad);

  py::class_<MotionConfig>(m, "MotionConfig")
      .def(py::init<>())
      .def_readwrite("action_name", &MotionConfig::action_name)
      .def_readwrite("pose_action_name", &MotionConfig::pose_action_name)
      .def_readwrite("pose_frame", &MotionConfig::pose_frame)
      .def_readwrite("joint_names", &MotionConfig::joint_names)
      .def_readwrite("default_timeout_s", &MotionConfig::default_timeout_s)
      .def_readwrite("named_targets", &MotionConfig::named_targets)
      .def_readwrite("gripper_joint", &MotionConfig::gripper_joint)
      .def_readwrite("gripper_open_position", &MotionConfig::gripper_open_position)
      .def_readwrite("gripper_closed_position", &MotionConfig::gripper_closed_position)
      .def_readwrite("gripper_timeout_s", &MotionConfig::gripper_timeout_s);

  py::class_<PerceptionConfig>(m, "PerceptionConfig")
      .def(py::init<>())
      .def_readwrite("laser_scan_topic", &PerceptionConfig::laser_scan_topic)
      .def_readwrite("imu_topic", &PerceptionConfig::imu_topic)
      .def_readwrite("depth_image_topic", &PerceptionConfig::depth_image_topic)
      .def_readwrite("point_cloud_topic", &PerceptionConfig::point_cloud_topic)
      .def_readwrite("camera_info_topic", &PerceptionConfig::camera_info_topic)
      .def_readwrite("sensor_timeout_s", &PerceptionConfig::sensor_timeout_s);

  py::class_<StatusConfig>(m, "StatusConfig")
      .def(py::init<>())
      .def_readwrite("status_topic", &StatusConfig::status_topic)
      .def_readwrite("joint_states_topic", &StatusConfig::joint_states_topic)
      .def_readwrite("get_status_service", &StatusConfig::get_status_service)
      .def_readwrite("default_timeout_s", &StatusConfig::default_timeout_s);

  // ── 传输层：QoS（与业务配置分开，见 docs/qos.md）──
  py::class_<SubscriptionQos>(m, "SubscriptionQos", "一个订阅的传输参数（QoS）。")
      .def(py::init<>())
      .def_readwrite("depth", &SubscriptionQos::depth)
      .def_readwrite("reliability", &SubscriptionQos::reliability);

  py::class_<QosConfig>(m, "QosConfig", "全局 QoS：一处配好、各模块按需取用。")
      .def(py::init<>())
      .def_readwrite("sensor", &QosConfig::sensor)
      .def_readwrite("joint_states", &QosConfig::joint_states)
      .def_readwrite("status", &QosConfig::status);

  py::class_<RobotLimitsConfig>(m, "RobotLimitsConfig",
                                "机器人自己的属性（到位判据 / 能力上限），由执行侧使用。")
      .def(py::init<>())
      .def_readwrite("goal_tolerance_m", &RobotLimitsConfig::goal_tolerance_m)
      .def_readwrite("yaw_tolerance_rad", &RobotLimitsConfig::yaw_tolerance_rad)
      .def_readwrite("max_speed", &RobotLimitsConfig::max_speed)
      .def_readwrite("max_yaw_rate", &RobotLimitsConfig::max_yaw_rate)
      .def_readwrite("joint_tolerance", &RobotLimitsConfig::joint_tolerance)
      .def_readwrite("default_joint_velocity",
                     &RobotLimitsConfig::default_joint_velocity)
      .def_readwrite("body_clearance_m", &RobotLimitsConfig::body_clearance_m);

  py::class_<RobotConfig>(m, "RobotConfig", "一个 Robot 的全部配置。")
      // 只给三个标量开构造参数（子配置字段多、名字也长，用属性赋值更清楚）。
      .def(py::init([](std::string node_name, std::string node_namespace, bool use_sim_time) {
             RobotConfig config;
             config.node_name = std::move(node_name);
             config.node_namespace = std::move(node_namespace);
             config.use_sim_time = use_sim_time;
             return config;
           }),
           py::arg("node_name") = "robot_sdk", py::arg("node_namespace") = "",
           py::arg("use_sim_time") = false)
      .def_readwrite("node_name", &RobotConfig::node_name)
      .def_readwrite("node_namespace", &RobotConfig::node_namespace)
      .def_readwrite("use_sim_time", &RobotConfig::use_sim_time)
      .def_readwrite("navigation", &RobotConfig::navigation)
      .def_readwrite("motion", &RobotConfig::motion)
      .def_readwrite("perception", &RobotConfig::perception)
      .def_readwrite("status", &RobotConfig::status)
      .def_readwrite("limits", &RobotConfig::limits)
      .def_readwrite("qos", &RobotConfig::qos);

  // ── 目标（客户给进去的）──
  py::class_<MotionGoal>(m, "MotionGoal", "关节空间目标。")
      .def(py::init<>())
      .def_readwrite("joint_names", &MotionGoal::joint_names)
      .def_readwrite("positions", &MotionGoal::positions)
      .def_readwrite("max_velocity", &MotionGoal::max_velocity)
      .def_readwrite("max_acceleration", &MotionGoal::max_acceleration)
      .def_readwrite("timeout_s", &MotionGoal::timeout_s);

  py::class_<PoseGoal>(m, "PoseGoal", "笛卡尔目标：把末端挪到某个位姿。")
      .def(py::init<>())
      .def_readwrite("any_orientation", &PoseGoal::any_orientation)
      .def_readwrite("max_velocity", &PoseGoal::max_velocity)
      .def_readwrite("timeout_s", &PoseGoal::timeout_s)
      // PoseStamped 是 ROS 消息，用 (x, y, z, qx, qy, qz, qw) 七元组表达更 Pythonic。
      .def_property(
          "target",
          [](const PoseGoal& goal) {
            return py::make_tuple(goal.target_pose.pose.position.x,
                                  goal.target_pose.pose.position.y,
                                  goal.target_pose.pose.position.z,
                                  goal.target_pose.pose.orientation.x,
                                  goal.target_pose.pose.orientation.y,
                                  goal.target_pose.pose.orientation.z,
                                  goal.target_pose.pose.orientation.w);
          },
          [](PoseGoal& goal, py::sequence values) {
            if (py::len(values) != 7) {
              throw py::value_error("target 需要 7 个分量：(x, y, z, qx, qy, qz, qw)");
            }
            goal.target_pose.pose.position.x = values[0].cast<double>();
            goal.target_pose.pose.position.y = values[1].cast<double>();
            goal.target_pose.pose.position.z = values[2].cast<double>();
            goal.target_pose.pose.orientation.x = values[3].cast<double>();
            goal.target_pose.pose.orientation.y = values[4].cast<double>();
            goal.target_pose.pose.orientation.z = values[5].cast<double>();
            goal.target_pose.pose.orientation.w = values[6].cast<double>();
          })
      .def_property(
          "frame",
          [](const PoseGoal& goal) { return goal.target_pose.header.frame_id; },
          [](PoseGoal& goal, const std::string& frame) {
            goal.target_pose.header.frame_id = frame;
          });

  py::class_<NavigationGoal>(m, "NavigationGoal", "高层导航目标。")
      .def(py::init<>())
      .def_readwrite("target", &NavigationGoal::target)
      .def_readwrite("frame", &NavigationGoal::frame)
      .def_readwrite("max_speed", &NavigationGoal::max_speed)
      .def_readwrite("avoid_obstacles", &NavigationGoal::avoid_obstacles)
      .def_readwrite("timeout_s", &NavigationGoal::timeout_s);

  // ── 句柄（两种 action 各一个类）──
  bindHandle<robot_sdk_interfaces::action::MoveToJointPositions>(
      m, "JointMotionHandle", "move_to_joint_positions 的句柄。");
  bindHandle<robot_sdk_interfaces::action::MoveToPose>(
      m, "PoseMotionHandle", "move_to_pose 的句柄。");
  bindHandle<robot_sdk_interfaces::action::NavigateToPose>(m, "NavigationHandle",
                                                          "navigate_to_pose 的句柄。");

  // ── 四个能力 API ──
  py::class_<MotionAPI>(m, "MotionAPI")
      .def(
          "server_available",
          [](MotionAPI& api, double timeout_s) {
            return withoutGil(
                [&] { return api.serverAvailable(std::chrono::duration<double>(timeout_s)); });
          },
          py::arg("timeout_s") = 5.0)
      .def(
          "move_to_joint_positions",
          [](MotionAPI& api, const MotionGoal& goal) {
            const auto result = withoutGil([&] { return api.moveToJointPositions(goal); });
            return unwrap(result);
          },
          py::arg("goal"))
      .def(
          "move_to_joint_positions_async",
          [](MotionAPI& api, const MotionGoal& goal) {
            return api.moveToJointPositionsAsync(goal);
          },
          py::arg("goal"))
      .def(
          "move_to_pose",
          [](MotionAPI& api, const PoseGoal& goal) {
            const auto result = withoutGil([&] { return api.moveToPose(goal); });
            return unwrap(result);
          },
          py::arg("goal"))
      .def(
          "move_to_pose_async",
          [](MotionAPI& api, const PoseGoal& goal) { return api.moveToPoseAsync(goal); },
          py::arg("goal"))
      .def(
          "move_to_named_target",
          [](MotionAPI& api, const std::string& name) {
            const auto result = withoutGil([&] { return api.moveToNamedTarget(name); });
            return unwrap(result);
          },
          py::arg("name"))
      .def(
          "home",
          [](MotionAPI& api) {
            const auto result = withoutGil([&] { return api.home(); });
            return unwrap(result);
          })
      .def(
          "set_gripper",
          [](MotionAPI& api, double position) {
            const auto result = withoutGil([&] { return api.setGripper(position); });
            return unwrap(result);
          },
          py::arg("position"))
      .def(
          "open_gripper",
          [](MotionAPI& api) {
            const auto result = withoutGil([&] { return api.openGripper(); });
            return unwrap(result);
          })
      .def(
          "close_gripper",
          [](MotionAPI& api) {
            const auto result = withoutGil([&] { return api.closeGripper(); });
            return unwrap(result);
          })
      .def("cancel_all", &MotionAPI::cancelAll);

  py::class_<NavigationAPI>(m, "NavigationAPI")
      .def(
          "server_available",
          [](NavigationAPI& api, double timeout_s) {
            return withoutGil(
                [&] { return api.serverAvailable(std::chrono::duration<double>(timeout_s)); });
          },
          py::arg("timeout_s") = 5.0)
      .def(
          "go_to",
          [](NavigationAPI& api, const NavigationGoal& goal) {
            const auto result = withoutGil([&] { return api.goTo(goal); });
            return unwrap(result);
          },
          py::arg("goal"))
      .def(
          "go_to_async",
          [](NavigationAPI& api, const NavigationGoal& goal) { return api.goToAsync(goal); },
          py::arg("goal"))
      .def(
          "current_pose",
          [](NavigationAPI& api, double timeout_s) {
            const auto result = withoutGil([&] {
              return api.currentPose(std::chrono::duration<double>(timeout_s));
            });
            return unwrap(result);
          },
          py::arg("timeout_s") = 0.1)
      .def("cancel_all", &NavigationAPI::cancelAll);

  py::class_<PerceptionAPI>(m, "PerceptionAPI")
      .def("has_laser_scan", &PerceptionAPI::hasLaserScan)
      .def("has_imu", &PerceptionAPI::hasImu)
      .def("has_depth_image", &PerceptionAPI::hasDepthImage)
      .def("has_camera_info", &PerceptionAPI::hasCameraInfo)
      .def("has_point_cloud", &PerceptionAPI::hasPointCloud)
      // 非阻塞读："还没有数据"返回 None（而**不是**抛异常）—— 它问的是"现在有吗"。
      .def("laser_scan",
           [](PerceptionAPI& api) { return projectOptional(api.laserScan()); })
      .def("imu", [](PerceptionAPI& api) { return projectOptional(api.imu()); })
      .def("depth_image",
           [](PerceptionAPI& api) { return projectOptional(api.depthImage()); })
      .def("camera_info",
           [](PerceptionAPI& api) { return projectOptional(api.cameraInfo()); })
      .def("point_cloud",
           [](PerceptionAPI& api) { return projectOptional(api.pointCloud()); })
      .def(
          "wait_for_laser_scan",
          [](PerceptionAPI& api, double timeout_s) {
            const auto result = withoutGil([&] {
              return api.waitForLaserScan(PerceptionAPI::Duration(timeout_s));
            });
            return unwrap(result);
          },
          py::arg("timeout_s") = 0.0)
      .def(
          "wait_for_imu",
          [](PerceptionAPI& api, double timeout_s) {
            const auto result =
                withoutGil([&] { return api.waitForImu(PerceptionAPI::Duration(timeout_s)); });
            return unwrap(result);
          },
          py::arg("timeout_s") = 0.0)
      .def(
          "wait_for_depth_image",
          [](PerceptionAPI& api, double timeout_s) {
            const auto result = withoutGil(
                [&] { return api.waitForDepthImage(PerceptionAPI::Duration(timeout_s)); });
            return unwrap(result);
          },
          py::arg("timeout_s") = 0.0)
      .def(
          "wait_for_camera_info",
          [](PerceptionAPI& api, double timeout_s) {
            const auto result = withoutGil(
                [&] { return api.waitForCameraInfo(PerceptionAPI::Duration(timeout_s)); });
            return unwrap(result);
          },
          py::arg("timeout_s") = 0.0)
      .def(
          "wait_for_point_cloud",
          [](PerceptionAPI& api, double timeout_s) {
            const auto result = withoutGil(
                [&] { return api.waitForPointCloud(PerceptionAPI::Duration(timeout_s)); });
            return unwrap(result);
          },
          py::arg("timeout_s") = 0.0)
      .def("nearest_obstacle_distance", &PerceptionAPI::nearestObstacleDistance,
           py::arg("angle_min") = -std::numeric_limits<double>::infinity(),
           py::arg("angle_max") = std::numeric_limits<double>::infinity());

  py::class_<StatusAPI>(m, "StatusAPI")
      .def("has_status", &StatusAPI::hasStatus)
      // 判据是类外的自由函数（作用于一条消息）；实例方法读缓存 —— 名字不同，不会混。
      .def("allows_motion", &StatusAPI::allowsMotion)
      .def("allows_navigation", &StatusAPI::allowsNavigation)
      .def("status", [](StatusAPI& api) { return projectOptional(api.status()); })
      .def("joint_states",
           [](StatusAPI& api) { return projectOptional(api.jointStates()); })
      .def(
          "wait_for_status",
          [](StatusAPI& api, double timeout_s) {
            const auto result = withoutGil(
                [&] { return api.waitForStatus(std::chrono::duration<double>(timeout_s)); });
            return unwrap(result);
          },
          py::arg("timeout_s"))
      .def(
          "wait_for_joint_states",
          [](StatusAPI& api, double timeout_s) {
            const auto result = withoutGil([&] {
              return api.waitForJointStates(std::chrono::duration<double>(timeout_s));
            });
            return unwrap(result);
          },
          py::arg("timeout_s"))
      .def(
          "query_status",
          [](StatusAPI& api, double timeout_s) {
            const auto result = withoutGil(
                [&] { return api.queryStatus(std::chrono::duration<double>(timeout_s)); });
            return unwrap(result);
          },
          py::arg("timeout_s"))
      .def("is_healthy", &StatusAPI::isHealthy)
      .def("battery_percentage", &StatusAPI::batteryPercentage)
      .def(
          "on_status",
          [](StatusAPI& api, py::function callback) {
            api.onStatus(GuardedCallback(std::move(callback)).forStatus());
          },
          py::arg("callback"));

  // ── 入口 ──
  py::class_<Robot>(m, "Robot", "SDK 入口：一个对象装四个能力 API。")
      .def(py::init([](const RobotConfig& config) {
             // Python 侧不该先学 rclcpp.init() —— 建节点前替它做掉（已经初始化过就不动）。
             if (!rclcpp::ok()) {
               rclcpp::init(0, nullptr);
             }
             // 构造期没有 Result 通道，配置不合法只能以异常形式出来。rclcpp 抛的是它自己的类型
             // （到 Python 会变成 ValueError），这里翻成 SDK 的异常 ⇒ Python 侧只剩 RobotSdkError。
             try {
               return std::make_unique<Robot>(config);
             } catch (const SdkException&) {
               throw;
             } catch (const std::exception& e) {
               throw SdkException(Error::InvalidArgument(e.what()));
             }
           }),
           py::arg("config") = RobotConfig{})
      .def("navigation", &Robot::navigation, py::return_value_policy::reference_internal)
      .def("motion", &Robot::motion, py::return_value_policy::reference_internal)
      .def("perception", &Robot::perception, py::return_value_policy::reference_internal)
      .def("status", &Robot::status, py::return_value_policy::reference_internal)
      .def("config", &Robot::config, py::return_value_policy::reference_internal)
      .def("owns_executor", &Robot::ownsExecutor)
      // ⚠️ spin() 会阻塞，而且**必须放开 GIL** —— 否则后台线程转不动、阻塞式 API 全死锁。
      .def("spin", &Robot::spin, py::call_guard<py::gil_scoped_release>())
      .def("spin_some", &Robot::spinSome, py::arg("timeout_ms"),
           py::call_guard<py::gil_scoped_release>())
      .def("stop", &Robot::stop);
}
