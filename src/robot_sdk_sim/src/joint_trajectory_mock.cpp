// FollowJointTrajectory 的**测试替身**（真机上是 ros2_control 的 joint_trajectory_controller）。
// 单进程、纯 ROS 参数、时序确定 ⇒ 给 motion_chain 那条测试用；自己也发 /joint_states。
// ⚠️ 动作名和真货重复，两个不能同时起。   ros2 run robot_sdk_sim joint_trajectory_mock
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

namespace {

/// 把关节名列表拼成一行，用于错误信息（"收到 X / 本机 Y"）。
std::string joinNames(const std::vector<std::string>& names) {
  std::string out;
  for (const std::string& name : names) {
    if (!out.empty()) {
      out += ", ";
    }
    out += name;
  }
  return out;
}

} // namespace

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
                   "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint"});
    positions_.assign(joint_names_.size(), 0.0);

    timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    server_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // ⚠️ 默认**不发** /joint_states：与 mock_robot_node 同起时会出现两个发布者（两套数据 ✗）。
    //    单独用它（如 motion_chain.launch）时才由参数打开 —— 一个栈里只允许一个发布者。
    if (declare_parameter<bool>("publish_joint_states", false)) {
      joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);
    }
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
      const double from = segment == 0 ? start_positions_[k]
                                       : points_[segment - 1].positions[mapping_[k]];
      const double to = points_[segment].positions[mapping_[k]];
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
        positions_[k] = points_.back().positions[mapping_[k]];
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
    if (joint_pub_) {
      joint_pub_->publish(msg);
    }
  }

  // ── 服务端回调 ──

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                         std::shared_ptr<const JtcAction::Goal> goal) {
    const auto& trajectory = goal->trajectory;
    if (trajectory.points.empty()) {
      RCLCPP_WARN(get_logger(), "拒绝轨迹：一个点都没有");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // ⚠️ 按**名字**对齐，顺序不敏感 —— MoveIt 送出的关节名是按字母序排的，和我们本机
    //    顺序不同。真控制器也是按名字索引；要求顺序一致反而会拒掉**合法**轨迹（实测栽过）。
    std::vector<std::size_t> mapping;
    mapping.reserve(joint_names_.size());
    for (const std::string& name : joint_names_) {
      const auto it =
          std::find(trajectory.joint_names.begin(), trajectory.joint_names.end(), name);
      if (it == trajectory.joint_names.end()) {
        RCLCPP_WARN(get_logger(), "拒绝轨迹：缺少本机关节 '%s'\n  收到 [%s]\n  本机 [%s]",
                    name.c_str(), joinNames(trajectory.joint_names).c_str(),
                    joinNames(joint_names_).c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      mapping.push_back(static_cast<std::size_t>(it - trajectory.joint_names.begin()));
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
        for (std::size_t k = 0; k < mapping.size(); ++k) {
          if (std::abs(first[mapping[k]] - positions_[k]) > kStartTolerance) {
            start_mismatch = true;
          }
        }
        if (!start_mismatch) {
          points_ = trajectory.points;
          mapping_ = mapping;
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
  /// 收到的轨迹里，本机第 k 个关节排在它的第几个 —— 按名字对齐（顺序不敏感）。
  std::vector<std::size_t> mapping_;
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
