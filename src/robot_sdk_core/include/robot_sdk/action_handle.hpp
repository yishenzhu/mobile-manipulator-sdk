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

/// 指向"正在执行中的 action 目标"的非阻塞句柄。
///
/// SDK 交给客户的是它，而不是 rclcpp_action 的 goal handle ——
/// 这样客户代码就与底下的 Action 实现保持独立。
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

  /// 把句柄绑定到一个刚发出去的目标上。
  ///
  /// 为什么需要"默认构造 + 事后 bind"两步：API 层得先有了 handle 对象，
  /// 才能把它的回调塞进 SendGoalOptions；而 goal 的 future 要等 send 之后
  /// 才拿得到。顺序上绕不开。
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
  ///
  /// 关键：**先把回调拷出来、解锁，再调用**。
  /// 绝不能在持锁状态下调用用户回调 —— 它可能很慢，也可能反过来
  /// 调用本对象的方法（比如 cancel()），那就会**自死锁**
  /// （std::mutex 不可重入）。
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

  /// 请求取消目标。可从任意线程调用。
  void cancel() {
    auto client = client_.lock();
    if (!client || !goal_future_.valid()) {
      return;
    }
    // 非阻塞地检查 future 是否就绪。目标还没被接受时取不到 goal handle，
    // 此时不该硬取（shared_future::get() 会阻塞）。
    if (goal_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return;
    }
    if (auto goal_handle = goal_future_.get()) {
      client->async_cancel_goal(goal_handle);
    }
  }

  /// 最多等 `timeout` 拿到目标的终态结果。
  ///
  /// 注意两点设计：
  ///   1. 两个阶段（等接受、等结果）**共用一个 deadline**，
  ///      所以总耗时受 timeout 约束，而不是"两个 timeout 相加"。
  ///   2. 超时不是撒手不管，而是**主动取消** —— 否则服务器上的目标还在跑，
  ///      机器人还在动，而客户以为已经放弃了。这是安全问题，不只是资源问题。
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
    if (result_future.wait_until(deadline) != std::future_status::ready) {
      client->async_cancel_goal(goal_handle);
      return Error::Timeout("timed out waiting for the action result");
    }
    return result_future.get();
  }

 private:
  bool accepted() const { return goal_future_.get() != nullptr; }

  // 必须是 shared_future 而不是 future：
  //   - future::get() 只能调一次（且会把值移走）
  //   - shared_future::get() 可以反复调，每次返回同一个值
  //   - 而且 shared_future 可拷贝，能安全共享给多个线程/回调
  // 本类里 accepted() / waitForAcceptance() / wait() / cancel() 都要反复查
  // 同一个 goal，所以只能用它。
  std::shared_future<typename GoalHandle::SharedPtr> goal_future_;
  // weak_ptr 而不是 shared_ptr：Client 由 API 层拥有，handle 只是"借用"。
  // 若用 shared_ptr，客户只要握着 handle 就能延长 Client（乃至节点）的寿命，
  // 造成不该有的资源滞留。weak_ptr 精确表达了正确的所有权语义，
  // 并且 lock() 失败是可检测的 —— 所以下面能报 "action client expired"。
  std::weak_ptr<Client> client_;
  FeedbackCallback feedback_callback_;
  std::mutex mutex_;
};

}  // namespace robot_sdk
