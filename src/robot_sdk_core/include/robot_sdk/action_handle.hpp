#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <utility>

#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_sdk/error.hpp"
#include "robot_sdk/result.hpp"

namespace robot_sdk {

/// 指向"正在执行中的 action 目标"的非阻塞句柄。SDK 交给客户的是它而不是
/// rclcpp_action 的 goal handle，这样客户代码与底下的 Action 实现保持独立。
template <typename ActionT>
class ActionHandle {
 public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using Feedback = typename ActionT::Feedback;
  using FeedbackSharedPtr = std::shared_ptr<const Feedback>;
  using FeedbackCallback = std::function<void(FeedbackSharedPtr)>;
  using Client = rclcpp_action::Client<ActionT>;

  ActionHandle() = default;
  ActionHandle(std::shared_future<typename GoalHandle::SharedPtr> goal_future,
               std::weak_ptr<Client> client)
      : goal_future_(std::move(goal_future)), client_(std::move(client)) {}

  bool valid() const { return goal_future_.valid(); }

  /// 把句柄绑定到一个刚发出去的目标上。之所以是"默认构造 + 事后 bind"两步：
  /// 得先有 handle 才能把回调塞进 SendGoalOptions，而 future 要等 send 之后才有。
  void bind(std::shared_future<typename GoalHandle::SharedPtr> goal_future,
            std::weak_ptr<Client> client) {
    goal_future_ = std::move(goal_future);
    client_ = std::move(client);
  }

  void onFeedback(FeedbackCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    feedback_callback_ = std::move(callback);
  }

  /// 当 action server 发布反馈时，由 API 层调用。
  /// ⚠️ 先把回调拷出来、解锁，再调用 —— 持锁调用户回调会自死锁
  /// （它可能很慢，也可能反过来调 cancel()；std::mutex 不可重入）。
  void dispatchFeedback(FeedbackSharedPtr feedback) {
    FeedbackCallback callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      callback = feedback_callback_;
    }
    if (callback) {
      callback(std::move(feedback));
    }
  }

  /// 最多等 `timeout`，看 server 是接受还是拒绝了目标。
  template <typename Rep, typename Period>
  bool waitForAcceptance(const std::chrono::duration<Rep, Period>& timeout) {
    if (!goal_future_.valid()) {
      return false;
    }
    return goal_future_.wait_for(timeout) == std::future_status::ready && accepted();
  }

  /// 请求取消目标，并**等服务端的取消响应**（对齐 rclpy 的 cancel_goal()）。
  /// 返回"服务端是否受理了这次取消"（响应里列出了这个 goal）。
  /// 前提是目标已被接受 —— 本 SDK 的 goToAsync/moveToPoseAsync 都等到接受了才返回，
  /// 所以这条路径不会遇到"还没有 goal handle"的空窗。
  /// ⚠️ 不要从 executor 线程（某个回调里）调用：等响应要 executor 处理，等下去会死锁。
  bool cancel() {
    auto client = client_.lock();
    if (!client || !goal_future_.valid()) {
      return false;
    }
    if (goal_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return false;
    }
    auto goal_handle = goal_future_.get();
    if (!goal_handle) {
      return false;
    }
    auto response_future = client->async_cancel_goal(goal_handle);
    const auto response = response_future.get();
    if (!response) {
      return false;
    }
    const auto wanted = goal_handle->get_goal_id();
    for (const auto& goal_info : response->goals_canceling) {
      // goal_id 是 ROS 消息类型（没有 operator==），比它内部那个 16 字节数组。
      if (goal_info.goal_id.uuid == wanted) {
        return true;
      }
    }
    return false;
  }

  /// 最多等 `timeout` 拿到目标的终态结果。两个阶段共用一个 deadline（总耗时受 timeout
  /// 约束，不是两个相加）。⚠️ 超时不是撒手不管，而是**主动取消** —— 否则机器人还在动
  /// 而客户以为已经放弃了，那是安全问题，不只是资源问题。
  template <typename Rep, typename Period>
  Result<WrappedResult> wait(const std::chrono::duration<Rep, Period>& timeout) {
    using namespace std::chrono;
    if (!goal_future_.valid()) {
      return Error::Internal("action handle is not bound to a goal");
    }
    // 量时间间隔必须用 steady_clock（单调），不能用 system_clock ——
    // 后者会被 NTP 校时跳变，导致超时算错。
    const auto deadline = steady_clock::now() + duration_cast<steady_clock::duration>(timeout);

    if (goal_future_.wait_until(deadline) != std::future_status::ready) {
      cancel();
      return Error::Timeout("timed out waiting for the action goal to be accepted");
    }
    auto goal_handle = goal_future_.get();
    if (!goal_handle) {
      return Error::Unavailable("action goal was rejected by the server");
    }

    auto client = client_.lock();
    if (!client) {
      return Error::Internal("action client expired while waiting for the result");
    }

    auto result_future = client->async_get_result(goal_handle);
    const bool got_result = result_future.wait_until(deadline) == std::future_status::ready;
    // 结果一旦请求，这个目标就必然走向终态：放掉对 rclcpp_action goal handle 的引用。
    // ⚠️ 两条出口都要放：GoalHandle 存着反馈回调、回调又捕获了 this ⇒ 不放就成环，
    //    handle 永不析构，登记册里的 weak_ptr 永不失效 ⇒ 之后 cancelAll 会去取消一个
    //    早已结束的目标 ⇒ client 抛 "Goal handle is not known to this client"。
    goal_future_ = {};
    if (!got_result) {
      client->async_cancel_goal(goal_handle);
      return Error::Timeout("timed out waiting for the action result");
    }
    return result_future.get();
  }

 private:
  bool accepted() const { return goal_future_.get() != nullptr; }

  // 必须 shared_future 而不是 future：后者 get() 只能调一次且会把值移走，
  // 前者可反复调、可拷贝、能安全共享给多线程。本类的 accepted() /
  // waitForAcceptance() / wait() / cancel() 都要反复查同一个 goal。
  std::shared_future<typename GoalHandle::SharedPtr> goal_future_;
  // weak_ptr 而不是 shared_ptr：Client 归 API 层所有，handle 只是借用。用 shared_ptr 的话
  // 客户握着 handle 就能延长 Client（乃至节点）的寿命。weak_ptr 的 lock() 失败可检测，
  // 所以下面能报 "action client expired"。
  std::weak_ptr<Client> client_;
  FeedbackCallback feedback_callback_;
  std::mutex mutex_;
};

}  // namespace robot_sdk
