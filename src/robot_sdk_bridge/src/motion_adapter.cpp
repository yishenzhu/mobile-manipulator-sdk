// 把 SDK 的关节运动动作翻译成 ros2_control 的 FollowJointTrajectory。
// 桥接器：没有本地状态要推进，只做翻译和转发，不需要定时器。
// 下游离不开、SDK 契约里却没有的两样在本节点补：轨迹起点（取自 /joint_states）、完成度。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rcl_action/rcl_action.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <robot_sdk_interfaces/action/move_to_joint_positions.hpp>

#include "robot_sdk/detail/action_slot.hpp"

namespace robot_sdk::bridge {

class MotionAdapter : public rclcpp::Node {
 public:
  using SdkAction = robot_sdk_interfaces::action::MoveToJointPositions;
  using SdkGoalHandle = rclcpp_action::ServerGoalHandle<SdkAction>;
  using JtcAction = control_msgs::action::FollowJointTrajectory;
  using JtcGoalHandle = rclcpp_action::ClientGoalHandle<JtcAction>;

  /// 距离为零时的最短时长，避免生成一条时长为 0 的轨迹。
  static constexpr double kMinMoveTimeS = 0.5;

  explicit MotionAdapter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("motion_adapter", options) {
    sdk_action_name_ =
        declare_parameter<std::string>("sdk_action_name", "sdk/move_to_joint_positions");
    jtc_action_name_ = declare_parameter<std::string>(
        "jtc_action_name", "/joint_trajectory_controller/follow_joint_trajectory");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    default_max_velocity_ = declare_parameter<double>("default_max_velocity", 1.0);

    server_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    joint_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    client_ = rclcpp_action::create_client<JtcAction>(this, jtc_action_name_, client_group_);
    // best-effort 订阅对 reliable / best-effort 两种发布端都兼容 —— 真机上的
    // /joint_states 两种都有，所以不跟某一种绑死。
    rclcpp::SubscriptionOptions joint_options;
    joint_options.callback_group = joint_group_;
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { onJointState(msg); },
        joint_options);
    server_ = rclcpp_action::create_server<SdkAction>(
        this, sdk_action_name_,
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const SdkAction::Goal> goal) {
          return handleGoal(uuid, std::move(goal));
        },
        [this](const std::shared_ptr<SdkGoalHandle> handle) { return handleCancel(handle); },
        [this](const std::shared_ptr<SdkGoalHandle> handle) { handleAccepted(handle); },
        rcl_action_server_get_default_options(), server_group_);
  }

 private:
  // ── 当前关节角（轨迹起点 + 完成度的分母都要它）──

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    joint_names_ = msg->name;
    joint_positions_ = msg->position;
  }

  /// 把 `target` 里的关节名映射成**本机顺序**下标；-1 表示本机没有这个关节。
  std::vector<int> resolveJoints(const std::vector<std::string>& target_names) {
    std::vector<int> indices;
    indices.reserve(target_names.size());
    for (const auto& name : target_names) {
      const auto it = std::find(joint_names_.begin(), joint_names_.end(), name);
      indices.push_back(it == joint_names_.end() ? -1 : static_cast<int>(it - joint_names_.begin()));
    }
    return indices;
  }

  // ── 服务端回调 ──

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                         std::shared_ptr<const SdkAction::Goal> goal) {
    const auto& target = goal->target;
    if (target.name.empty() || target.name.size() != target.position.size()) {
      RCLCPP_WARN(get_logger(), "拒绝运动目标：关节名缺失或与位置数量不符");
      return rclcpp_action::GoalResponse::REJECT;
    }

    bool busy = false;
    std::vector<int> indices;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      busy = slot_.busy();
      if (!busy) {
        // 起点在这一刻定死：后面反馈的完成度全以它为分母。
        if (joint_positions_.empty()) {
          RCLCPP_WARN(get_logger(), "还没收到 %s，无法确定轨迹起点",
                      joint_states_topic_.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        indices = resolveJoints(target.name);
        for (std::size_t i = 0; i < indices.size(); ++i) {
          if (indices[i] < 0) {
            RCLCPP_WARN(get_logger(), "拒绝运动目标：本机没有关节 '%s'",
                        target.name[i].c_str());
            return rclcpp_action::GoalResponse::REJECT;
          }
        }
        slot_.reserve(); // 占位：下游句柄要等回应回调才到
        start_positions_.clear();
        for (int index : indices) {
          start_positions_.push_back(joint_positions_[index]);
        }
        target_positions_ = target.position;
        initial_error_ = maxError(start_positions_, target_positions_);
      }
    }
    if (busy) {
      RCLCPP_WARN(get_logger(), "拒绝运动目标：上一个还没结束");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<SdkGoalHandle> sdk_handle) {
    if (!sdk_handle->is_active()) { // 问框架在锁外
      return rclcpp_action::CancelResponse::REJECT;
    }

    std::shared_ptr<JtcGoalHandle> jtc_handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slot_.cancel_requested = true;
      jtc_handle = jtc_handle_;
    }
    // 下游句柄已经到手就立刻转发；还没到的话，等目标回应回调里补发。
    if (jtc_handle) {
      client_->async_cancel_goal(jtc_handle);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<SdkGoalHandle> sdk_handle) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slot_.begin(sdk_handle);
    }

    auto jtc_goal = std::make_shared<JtcAction::Goal>();
    buildTrajectory(*sdk_handle->get_goal(), *jtc_goal);

    rclcpp_action::Client<JtcAction>::SendGoalOptions options;
    options.goal_response_callback = [this](const std::shared_ptr<JtcGoalHandle> handle) {
      onJtcGoalResponse(handle);
    };
    options.feedback_callback = [this](std::shared_ptr<JtcGoalHandle>,
                                       const std::shared_ptr<const JtcAction::Feedback> feedback) {
      onJtcFeedback(feedback);
    };
    options.result_callback = [this](const JtcGoalHandle::WrappedResult& wrapped) {
      onJtcResult(wrapped);
    };

    client_->async_send_goal(*jtc_goal, options);
  }

  /// 一个目标点 → 一条两点轨迹（当前 → 目标）。
  /// 时长取**所有关节里最慢的那个**，于是所有关节同时到位。
  void buildTrajectory(const SdkAction::Goal& goal, JtcAction::Goal& out) {
    std::vector<double> start;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      start = start_positions_;
    }

    const double velocity =
        goal.max_velocity > 0.0F ? goal.max_velocity : default_max_velocity_;
    double duration = kMinMoveTimeS;
    for (std::size_t i = 0; i < goal.target.position.size(); ++i) {
      const double distance = goal.target.position[i] - start[i];
      duration = std::max(duration, moveTime(distance, velocity, goal.max_acceleration));
    }

    out.trajectory.joint_names = goal.target.name;
    out.trajectory.header.stamp = now();

    out.trajectory.points.resize(2);
    out.trajectory.points[0].positions = start;
    out.trajectory.points[0].time_from_start = toDuration(0.0);
    out.trajectory.points[1].positions = goal.target.position;
    out.trajectory.points[1].time_from_start = toDuration(duration);
  }

  /// 走完 `distance` 所需的时间。两项分别是"速度允许的"和"加速度允许的"
  /// （后者是三角速度曲线），取大者即同时满足两个约束。
  static double moveTime(double distance, double velocity, double acceleration) {
    const double by_velocity = velocity > 0.0 ? std::abs(distance) / velocity : 0.0;
    const double by_acceleration = acceleration > 0.0
                                       ? 2.0 * std::sqrt(std::abs(distance) / acceleration)
                                       : 0.0;
    return std::max(by_velocity, by_acceleration);
  }

  static builtin_interfaces::msg::Duration toDuration(double seconds) {
    builtin_interfaces::msg::Duration out;
    out.sec = static_cast<int32_t>(seconds);
    out.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(out.sec)) * 1e9);
    return out;
  }

  /// 各关节误差的**最大值** —— SDK 只给一个标量，取最大是唯一不撒谎的合成方式。
  static double maxError(const std::vector<double>& actual,
                         const std::vector<double>& target) {
    double worst = 0.0;
    for (std::size_t i = 0; i < actual.size() && i < target.size(); ++i) {
      worst = std::max(worst, std::abs(actual[i] - target[i]));
    }
    return worst;
  }

  // ── 下游客户端的三个回调 ──

  void onJtcGoalResponse(const std::shared_ptr<JtcGoalHandle> jtc_handle) {
    std::shared_ptr<SdkGoalHandle> sdk_handle;
    bool cancel_late = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sdk_handle = slot_.handle;
      if (jtc_handle) {
        jtc_handle_ = jtc_handle;
        // 取消可能比下游句柄先到 —— 那时转发不了，这里补上。
        cancel_late = slot_.cancel_requested;
      }
    }

    if (!jtc_handle) {
      if (sdk_handle) {
        auto result = std::make_shared<SdkAction::Result>();
        result->success = false;
        result->message = "关节轨迹控制器拒绝了目标";
        sdk_handle->abort(result);
      }
      std::lock_guard<std::mutex> lock(mutex_);
      slot_.finish();
      return;
    }
    if (cancel_late) {
      client_->async_cancel_goal(jtc_handle);
    }
  }

  void onJtcFeedback(const std::shared_ptr<const JtcAction::Feedback> feedback) {
    std::shared_ptr<SdkGoalHandle> sdk_handle;
    double initial_error = 0.0;
    double error = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sdk_handle = slot_.handle;
      initial_error = initial_error_;
      error = maxError(feedback->actual.positions, target_positions_);
      last_error_ = static_cast<float>(error);
    }
    if (!sdk_handle || !sdk_handle->is_active()) {
      return;
    }

    auto out = std::make_shared<SdkAction::Feedback>();
    out->current.name = feedback->joint_names;
    out->current.position = feedback->actual.positions;
    // 分母是**目标下达时**的误差：它若为 0，这个目标本来就不需要动。
    // 量纲是**百分数**（与 mock 一致），不是 0..1。
    out->completion_percentage = initial_error > 0.0
                                     ? static_cast<float>(100.0 * (1.0 - error / initial_error))
                                     : 100.0F;
    sdk_handle->publish_feedback(out);
  }

  void onJtcResult(const JtcGoalHandle::WrappedResult& wrapped) {
    std::shared_ptr<SdkGoalHandle> sdk_handle;
    float last_error = 0.0F;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sdk_handle = slot_.handle;
      last_error = last_error_;
      slot_.finish();
      jtc_handle_.reset();
    }
    if (!sdk_handle) {
      return;
    }

    auto result = std::make_shared<SdkAction::Result>();
    result->final_position_error = last_error;
    if (wrapped.result) {
      result->success = wrapped.result->error_code == JtcAction::Result::SUCCESSFUL;
      result->message = wrapped.result->error_string;
    }

    // 终态**由下游决定**，本节点不自己判。
    // ⚠️ 不能只看 error_code：下游 abort 时它可能仍是 SUCCESSFUL 之外的常见值 0 ——
    //    那样"被中止的目标"会被报成 success。终态码说什么就是什么。
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      result->success = false;
      if (result->message.empty()) {
        result->message = "运动被取消";
      }
      sdk_handle->canceled(result);
    } else if (wrapped.code == rclcpp_action::ResultCode::ABORTED && result->success) {
      result->success = false;
      if (result->message.empty()) {
        result->message = "运动被中止";
      }
      sdk_handle->abort(result);
    } else if (result->success) {
      sdk_handle->succeed(result);
    } else {
      sdk_handle->abort(result);
    }
  }

  std::string sdk_action_name_;
  std::string jtc_action_name_;
  std::string joint_states_topic_;
  double default_max_velocity_ = 1.0;

  rclcpp::CallbackGroup::SharedPtr server_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::CallbackGroup::SharedPtr joint_group_;
  rclcpp_action::Server<SdkAction>::SharedPtr server_;
  rclcpp_action::Client<JtcAction>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  /// 保护下面这些成员。回调分落在三条可重入车道上，所以它们真的会并发。
  std::mutex mutex_;
  /// 我方目标的生命周期（三态 + 我方句柄 + 取消信箱）。
  ActionSlot<SdkAction> slot_;
  std::shared_ptr<JtcGoalHandle> jtc_handle_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  /// 目标下达那一刻的各关节角 —— 轨迹起点（每条轨迹只发一次）。
  std::vector<double> start_positions_;
  /// 目标位置 —— 完成度的分子拿它算。**不能用下游反馈里的 `desired`**：
  /// 那是"此刻应该在哪"，追踪正常时它和实际位置几乎重合，误差恒为 0。
  std::vector<double> target_positions_;
  /// 目标下达那一刻的最大误差 —— 完成度的分母。
  double initial_error_ = 0.0;
  /// 最后一条反馈里的最大误差 —— 下游结果里没有这个字段，只能用它。
  float last_error_ = std::numeric_limits<float>::infinity();
};

} // namespace robot_sdk::bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<robot_sdk::bridge::MotionAdapter>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
