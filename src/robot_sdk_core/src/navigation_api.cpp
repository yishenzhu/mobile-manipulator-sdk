#include "robot_sdk/navigation_api.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "robot_sdk/detail/action_util.hpp"

namespace robot_sdk {

NavigationAPI::NavigationAPI(
    const rclcpp::Node::SharedPtr &node,
    const rclcpp::CallbackGroup::SharedPtr &callback_group,
    std::shared_ptr<TransformManager> transform, const NavigationConfig &config)
    : node_(node), config_(config), transform_(std::move(transform)) {
  // 相对动作名，由 rcl
  // 按节点命名空间展开；回调组决定目标/结果回调落在哪条车道。
  client_ = rclcpp_action::create_client<Action>(node_, config_.action_name,
                                                 callback_group);
}

bool NavigationAPI::serverAvailable(std::chrono::duration<double> timeout) {
  return client_->wait_for_action_server(timeout);
}

NavigationAPI::Action::Goal
NavigationAPI::makeGoalMessage(const NavigationGoal &goal) const {
  Action::Goal msg;
  msg.target_pose.header.frame_id =
      goal.frame.empty() ? config_.global_frame : goal.frame;
  msg.target_pose.header.stamp = node_->now();
  msg.target_pose.pose = toPoseMsg(goal.target);
  msg.max_speed = static_cast<float>(goal.max_speed);
  msg.avoid_obstacles = goal.avoid_obstacles;
  return msg;
}

std::shared_ptr<ActionHandle<NavigationAPI::Action>>
NavigationAPI::goToAsync(const NavigationGoal &goal,
                         FeedbackCallback on_feedback) {
  auto handle = std::make_shared<ActionHandle<Action>>();
  if (on_feedback) {
    // 把 SDK 的 feedback 类型翻译成客户的类型 ——
    // 客户永远看不到 action 消息里那一堆字段。
    handle->onFeedback(
        [on_feedback](ActionHandle<Action>::FeedbackSharedPtr msg) {
          NavigationFeedback feedback;
          feedback.current_pose = toPose2D(msg->current_pose);
          feedback.distance_remaining = msg->distance_remaining;
          feedback.estimated_time_remaining = msg->estimated_time_remaining;
          feedback.state = msg->current_state;
          on_feedback(feedback);
        });
  }

  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback =
      [handle](typename ActionHandle<Action>::GoalHandle::SharedPtr,
               ActionHandle<Action>::FeedbackSharedPtr msg) {
        handle->dispatchFeedback(std::move(msg));
      };

  handle->bind(client_->async_send_goal(makeGoalMessage(goal), options),
               client_);

  // 对齐 nav2（`if not accepted: return None`）：等到"接受"才把 handle 交出去；没等到就**不交**。
  // 交出去的话调用方的 cancel() 会静默失效 —— 那正是这次要修的洞。
  if (!handle->waitForAcceptance(std::chrono::duration<double>(config_.server_wait_timeout_s))) {
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(handles_mutex_);
  active_handles_.push_back(handle);
  return handle;
}

NavigationResult NavigationAPI::toResult(
    const typename ActionHandle<Action>::WrappedResult &wrapped) {
  NavigationResult out;
  if (wrapped.result) {
    out.success = wrapped.result->success;
    out.final_distance_error = wrapped.result->final_distance_error;
    out.message = wrapped.result->message;
  }
  // 又是那两层：action 的终态码 != 结果里的 success 标志。
  // 有时候结果体是空的（比如被 abort），这时靠终态码补出可读信息。
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    out.success = false;
    if (out.message.empty()) {
      out.message =
          "goal terminated with code " + resultCodeToString(wrapped.code);
    }
  }
  return out;
}

Result<NavigationResult> NavigationAPI::goTo(const NavigationGoal &goal) {
  const double timeout =
      goal.timeout_s > 0.0 ? goal.timeout_s : config_.default_timeout_s;
  auto handle = goToAsync(goal);
  if (!handle) {
    // 目标没被接受（超时或服务端拒收）—— 派发阶段就失败了，没有句柄可用。
    return Error::Unavailable("action server did not accept the goal");
  }
  auto wrapped = handle->wait(std::chrono::duration<double>(timeout));
  if (!wrapped.ok()) {
    return wrapped.error();
  }
  auto result = toResult(wrapped.value());

  // 兑现头文件里那句承诺：success 要是我们**核过**的，不是照抄服务端的自称。
  // 读不回位姿就不改判 —— 不能因为一次查询失败把已经成功的动作判死。
  if (result.success) {
    auto pose = currentPose(std::chrono::duration<double>(config_.server_wait_timeout_s));
    if (pose.ok()) {
      const double distance =
          std::hypot(pose.value().x - goal.target.x, pose.value().y - goal.target.y);
      const double yaw_error =
          std::abs(normalizeAngle(pose.value().yaw - goal.target.yaw));
      if (distance > config_.arrival_tolerance_m ||
          yaw_error > config_.arrival_yaw_tolerance_rad) {
        result.success = false;
        result.message = "reported success but actual pose is off: distance " +
                         std::to_string(distance) + " m, yaw " +
                         std::to_string(yaw_error) + " rad";
      }
    }
  }
  return result;
}

Result<Pose2D>
NavigationAPI::currentPose(std::chrono::duration<double> timeout) const {
  return transform_->currentPose(config_.global_frame, config_.base_frame,
                                 timeout);
}

void NavigationAPI::cancelAll() {
  // 锁内只做"清理登记 + 把句柄拷出来"这两件小事，**取消动作放到锁外** ——
  // cancel() 会发取消请求并等响应（有 I/O 和等待），不该占着这把锁。最坏 N 个句柄 × 200 ms。
  // 这与本项目其它地方（dispatchFeedback / onStatus）是同一个模式。
  std::vector<std::shared_ptr<ActionHandle<Action>>> handles;
  {
    std::lock_guard<std::mutex> lock(handles_mutex_);

    // 顺手清掉已过期的登记（客户已经丢掉句柄的那些），否则这个向量只增不减。
    active_handles_.erase(
        std::remove_if(active_handles_.begin(), active_handles_.end(),
                       [](const std::weak_ptr<ActionHandle<Action>> &weak) {
                         return weak.expired();
                       }),
        active_handles_.end());

    // weak.lock() 拿到的是**强引用**：对象从此由 handles 保活，即使客户在另一线程丢掉
    // 自己的 shared_ptr 也不会在锁外析构（把 weak_ptr 拷进 vector 再锁外 lock 就会）。
    for (auto &weak : active_handles_) {
      if (auto handle = weak.lock()) {
        handles.push_back(std::move(handle));
      }
    }
  }

  for (auto &handle : handles) {
    handle->cancel();
  }
}

} // namespace robot_sdk
