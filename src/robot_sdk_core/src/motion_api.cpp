#include "robot_sdk/motion_api.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robot_sdk/detail/action_util.hpp"

namespace robot_sdk {

MotionAPI::MotionAPI(const rclcpp::Node::SharedPtr& node,
                     const rclcpp::CallbackGroup::SharedPtr& callback_group,
                     const MotionConfig& config)
    : node_(node), config_(config) {
  // 相对动作名 + 显式回调组（决定目标/结果回调落在哪条车道）。
  client_ = rclcpp_action::create_client<Action>(node_, config_.action_name, callback_group);
}

bool MotionAPI::serverAvailable(std::chrono::duration<double> timeout) {
  return client_->wait_for_action_server(timeout);
}

Error MotionAPI::validate(const MotionGoal& goal) const {
  // 只查**与具体机器人无关**的那部分：名字和位置必须一一对应。
  // 再做一份"这关节能转到哪"是**不收敛**的 —— 那是机器人的属性，
  // 由它自己在收到目标时判（见 config.hpp 里 MotionConfig 的说明）。
  if (goal.joint_names.empty()) {
    return Error::InvalidArgument("motion goal has no joint names");
  }
  if (goal.joint_names.size() != goal.positions.size()) {
    return Error::InvalidArgument("motion goal has " + std::to_string(goal.joint_names.size()) +
                                  " joint names but " + std::to_string(goal.positions.size()) +
                                  " positions");
  }
  return Error::Ok();
}

std::shared_ptr<ActionHandle<MotionAPI::Action>> MotionAPI::moveToJointPositionsAsync(
    const MotionGoal& goal, FeedbackCallback on_feedback) {
  auto handle = std::make_shared<ActionHandle<Action>>();
  if (on_feedback) {
    // 把 action 的 feedback 翻译成 SDK 的类型。
    handle->onFeedback([on_feedback](ActionHandle<Action>::FeedbackSharedPtr msg) {
      MotionFeedback feedback;
      feedback.joint_names = msg->current.name;
      feedback.positions = msg->current.position;
      feedback.completion_percentage = msg->completion_percentage;
      on_feedback(feedback);
    });
  }

  Action::Goal message;
  message.target.name = goal.joint_names;
  message.target.position = goal.positions;
  // max_velocity 照原样透传：<= 0 表示"不指定"，由机器人按自己的上限决定。
  message.max_velocity = static_cast<float>(goal.max_velocity);
  message.max_acceleration = static_cast<float>(goal.max_acceleration);

  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback = [handle](typename ActionHandle<Action>::GoalHandle::SharedPtr,
                                       ActionHandle<Action>::FeedbackSharedPtr msg) {
    handle->dispatchFeedback(std::move(msg));
  };

  handle->bind(client_->async_send_goal(message, options), client_);

  std::lock_guard<std::mutex> lock(handles_mutex_);
  active_handles_.push_back(handle);
  return handle;
}

Result<MotionResult> MotionAPI::moveToJointPositions(const MotionGoal& goal) {
  // 参数错误在**发出去之前**就拦掉 —— 比等后端拒绝更快、也更清楚。
  auto valid = validate(goal);
  if (!valid.ok()) {
    return valid;  // Error 隐式转成 Result<MotionResult>（同一套错误类型）
  }

  const double timeout = goal.timeout_s > 0.0 ? goal.timeout_s : config_.default_timeout_s;

  auto handle = moveToJointPositionsAsync(goal);
  auto wrapped = handle->wait(std::chrono::duration<double>(timeout));
  if (!wrapped.ok()) {
    return wrapped.error();
  }

  MotionResult result;
  if (wrapped.value().result) {
    result.success = wrapped.value().result->success;
    result.final_position_error = wrapped.value().result->final_position_error;
    result.message = wrapped.value().result->message;
  }
  // 又是两层：action 终态码 != 结果里的 success。
  if (wrapped.value().code != rclcpp_action::ResultCode::SUCCEEDED) {
    result.success = false;
    if (result.message.empty()) {
      result.message = "goal terminated with code " + resultCodeToString(wrapped.value().code);
    }
  }
  return result;
}

Result<MotionResult> MotionAPI::moveToNamedTarget(const std::string& name) {
  const auto it = config_.named_targets.find(name);
  if (it == config_.named_targets.end()) {
    return Error::InvalidArgument("unknown named target '" + name + "'");
  }
  if (config_.joint_names.empty()) {
    // 没有关节名就无法把"一串位置"对应到具体关节 ——
    // 这正是当初把 joint_names 默认值留空的原因（默认值不能撒谎）。
    return Error::NotInitialized("motion.joint_names is empty; cannot resolve named target");
  }
  MotionGoal goal;
  goal.joint_names = config_.joint_names;
  goal.positions = it->second;
  // 位置个数与关节名个数不一致时，由 validate() 拦下。
  return moveToJointPositions(goal);
}

Result<MotionResult> MotionAPI::home() { return moveToNamedTarget("home"); }

Result<MotionResult> MotionAPI::setGripper(double position) {
  MotionGoal goal;
  goal.joint_names = {config_.gripper_joint};
  goal.positions = {position};
  goal.timeout_s = config_.gripper_timeout_s;
  return moveToJointPositions(goal);
}

Result<MotionResult> MotionAPI::openGripper() {
  return setGripper(config_.gripper_open_position);
}

Result<MotionResult> MotionAPI::closeGripper() {
  return setGripper(config_.gripper_closed_position);
}

void MotionAPI::cancelAll() {
  // 与 NavigationAPI::cancelAll() 同一模式：
  // 锁内只做"清理登记 + 拷出句柄"，取消动作放锁外（cancel() 含 I/O）。
  std::vector<std::shared_ptr<ActionHandle<Action>>> handles;
  {
    std::lock_guard<std::mutex> lock(handles_mutex_);

    // 清掉已过期的登记，否则这个向量只增不减。
    active_handles_.erase(std::remove_if(active_handles_.begin(), active_handles_.end(),
                                        [](const std::weak_ptr<ActionHandle<Action>>& weak) {
                                          return weak.expired();
                                        }),
                          active_handles_.end());

    // lock() 拿到强引用 → 对象由 handles 保活，锁外使用安全。
    for (auto& weak : active_handles_) {
      if (auto handle = weak.lock()) {
        handles.push_back(std::move(handle));
      }
    }
  }

  for (auto& handle : handles) {
    handle->cancel();
  }
}

}  // namespace robot_sdk
