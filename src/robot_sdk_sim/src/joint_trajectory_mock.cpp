// 假的“关节轨迹控制器” —— ros2_control 的 joint_trajectory_controller 的替身。
//
// 它只说 **FollowJointTrajectory**，并且自己发 `/joint_states`。
// 用途是验 SDK 的运动链路：`motion_adapter` 的下游就是它。
// 真机上这个角色由 ros2_control 扮演（本机没装，所以有这个替身）。
//
// 行为刻意贴近真的控制器：
//   · 轨迹首点必须贴合**当前状态**，否则拒收
//   · 按 points 里的 `time_from_start` 逐点线性插值
//   · 反馈里的 `desired` 报的是**此刻应该在哪**（轨迹上的插值点），不是最终目标
//
//   ros2 run robot_sdk_sim joint_trajectory_mock
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rcl_action/rcl_action.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "robot_sdk_sim/tick_action.hpp"

namespace robot_sdk::sim {

using JtcAction = control_msgs::action::FollowJointTrajectory;

class JointTrajectoryMock : public rclcpp::Node {
 public:
  static constexpr double kPeriodS = 0.02;      // 控制周期 50 Hz
  static constexpr int kFeedbackEveryNTicks = 3; // 反馈 ≈16 Hz
  /// 轨迹首点与当前状态的最大允许偏差（弧度 / 米）。
  static constexpr double kStartTolerance = 0.1;

  explicit JointTrajectoryMock(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("joint_trajectory_mock", options) {
    action_name_ = declare_parameter<std::string>(
        "action_name", "/joint_trajectory_controller/follow_joint_trajectory");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joints", {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint",
                   "gripper_joint"});
    positions_.assign(joint_names_.size(), 0.0);

    timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    server_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);
    // 控制回路与状态发布同在一条互斥车道上，彼此不并发（同 mock_robot_node）。
    tick_timer_ = create_wall_timer(std::chrono::duration<double>(kPeriodS),
                                    [this]() { tick(); }, timer_group_);

    server_ = rclcpp_action::create_server<JtcAction>(
        this, action_name_,
        [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const JtcAction::Goal> goal) {
          return handleGoal(uuid, std::move(goal));
        },
        [this](const std::shared_ptr<GoalHandle> handle) { return handleCancel(handle); },
        [this](const std::shared_ptr<GoalHandle> handle) { handleAccepted(handle); },
        rcl_action_server_get_default_options(), server_group_);

    RCLCPP_INFO(get_logger(), "假关节轨迹控制器就绪：%zu 个关节，动作 '%s'",
                joint_names_.size(), action_name_.c_str());
  }

 private:
  using GoalHandle = rclcpp_action::ServerGoalHandle<JtcAction>;

  // ── 每拍 ──

  void tick() {
    publishJointStates(); // 骨架不管这个：它只做"推进 + 派发"
    tickAction<JtcAction>(mutex_, slot_,
                          [this](std::lock_guard<std::mutex>& lock) -> TickOutput<JtcAction> {
                            return stepTrajectory(lock);
                          });
  }

  /// 沿 trajectory 的 points 逐点线性插值。填上 result 即表示走完了。
  /// 参数 `lock` 是凭证（它读 `points_` / 写 `positions_`）。
  TickOutput<JtcAction> stepTrajectory(std::lock_guard<std::mutex>& /*lock*/) {
    TickOutput<JtcAction> output;

    elapsed_s_ += kPeriodS;
    const double t = elapsed_s_;

    // 找到当前所处的区间：[i-1, i]
    std::size_t segment = 1;
    while (segment + 1 < points_.size() && toSeconds(points_[segment].time_from_start) < t) {
      ++segment;
    }
    const double t0 = segment == 0 ? 0.0 : toSeconds(points_[segment - 1].time_from_start);
    const double t1 = toSeconds(points_[segment].time_from_start);
    const double ratio = t1 > t0 ? std::clamp((t - t0) / (t1 - t0), 0.0, 1.0) : 1.0;

    for (std::size_t k = 0; k < positions_.size(); ++k) {
      const double from = segment == 0 ? start_positions_[k] : points_[segment - 1].positions[k];
      const double to = points_[segment].positions[k];
      positions_[k] = from + (to - from) * ratio;
    }

    if (++feedback_divider_ >= kFeedbackEveryNTicks) {
      feedback_divider_ = 0;
      auto feedback = std::make_shared<JtcAction::Feedback>();
      feedback->header.stamp = now();
      feedback->joint_names = joint_names_;
      // desired 报"此刻应该在哪" —— 真的控制器就是这么报的。
      feedback->desired.positions = positions_;
      feedback->actual.positions = positions_;
      feedback->error.positions.assign(positions_.size(), 0.0);
      output.feedback = feedback;
    }

    if (t >= toSeconds(points_.back().time_from_start)) {
      for (std::size_t k = 0; k < positions_.size(); ++k) {
        positions_[k] = points_.back().positions[k];
      }
      auto result = std::make_shared<JtcAction::Result>();
      result->error_code = JtcAction::Result::SUCCESSFUL;
      output.result = result;
    }
    return output;
  }

  void publishJointStates() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg.name = joint_names_;
      msg.position = positions_;
    }
    msg.velocity.assign(joint_names_.size(), 0.0);
    msg.effort.assign(joint_names_.size(), 0.0);
    joint_pub_->publish(msg);
  }

  // ── 服务端回调 ──

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                         std::shared_ptr<const JtcAction::Goal> goal) {
    const auto& trajectory = goal->trajectory;
    if (trajectory.points.empty()) {
      RCLCPP_WARN(get_logger(), "拒绝轨迹：一个点都没有");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (trajectory.joint_names != joint_names_) {
      RCLCPP_WARN(get_logger(), "拒绝轨迹：关节名与本机不符");
      return rclcpp_action::GoalResponse::REJECT;
    }

    bool busy = false;
    bool start_mismatch = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      busy = slot_.busy();
      if (!busy) {
        // 真的控制器会检查"轨迹首点是否贴合当前状态" —— 不贴合就没法跟。
        start_positions_ = positions_;
        const auto& first = trajectory.points.front().positions;
        for (std::size_t k = 0; k < first.size() && k < positions_.size(); ++k) {
          if (std::abs(first[k] - positions_[k]) > kStartTolerance) {
            start_mismatch = true;
          }
        }
        if (!start_mismatch) {
          points_ = trajectory.points;
          elapsed_s_ = 0.0;
          feedback_divider_ = 0;
          slot_.reserve();
        }
      }
    }
    if (busy) {
      RCLCPP_WARN(get_logger(), "拒绝轨迹：上一条还在走");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (start_mismatch) {
      RCLCPP_WARN(get_logger(), "拒绝轨迹：首点与当前状态差得太多");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> handle) {
    if (!handle->is_active()) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    slot_.cancel_requested = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    slot_.begin(handle); // 句柄与相位同时改变（同 mock_robot_node）
  }

  static double toSeconds(const builtin_interfaces::msg::Duration& d) {
    return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
  }

  std::string action_name_;
  std::string joint_states_topic_;
  std::vector<std::string> joint_names_;

  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::CallbackGroup::SharedPtr server_group_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp_action::Server<JtcAction>::SharedPtr server_;

  /// 保护下面这些成员。控制回路与动作回调分落在两条车道上，真的会并发。
  std::mutex mutex_;
  /// 我校目标的"相位 + 句柄 + 取消信箱"。
  ActionSlot<JtcAction> slot_;
  std::vector<double> positions_;
  std::vector<double> start_positions_;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points_;
  double elapsed_s_{0.0};
  int feedback_divider_{0};
};

} // namespace robot_sdk::sim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<robot_sdk::sim::JointTrajectoryMock>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
