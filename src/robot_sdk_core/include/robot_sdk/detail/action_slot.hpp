#pragma once

#include <memory>
#include <mutex>
#include <utility>

#include <rclcpp_action/rclcpp_action.hpp>

namespace robot_sdk {

/// 一个动作服务端的槽位：把一个目标的生命周期收成**一个值**。
/// 不用三个 bool —— 8 种组合里只有 4 种合法，而三态枚举让非法组合无法表达。
/// 同时只服务一个同类目标的服务端，每个动作一个槽位。
template <typename Action>
struct ActionSlot {
  enum class Phase {
    kIdle,     ///< 没有目标。
    kReserved, ///< 目标已被接受，但 `handle` 还没到（框架在两个回调之间）。
    kRunning,  ///< 句柄到位，正在跑。
  };

  Phase phase{Phase::kIdle};

  /// 只在 `kRunning` 有效 —— 这正是要分出 `kReserved` 的原因：
  /// 目标已被接受时它**还是空的**，此时拿它去回调框架就是空指针解引用。
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> handle;

  /// **一位信箱**，不是状态：取消回调（框架线程）投递，服务端的执行侧取走。
  /// 它**任何相位都可能到达**（连 `kIdle` 都可能 —— 取消回调里 `is_active()`
  /// 与这里加锁之间目标可能已结束），所以不能并进 `Phase`。
  bool cancel_requested{false};

  bool busy() const { return phase != Phase::kIdle; }
  bool running() const { return phase == Phase::kRunning; }

  /// 占位：挡住并发到达的第二个目标。**不置 `kRunning`** —— 句柄还没到。
  void reserve() {
    phase = Phase::kReserved;
    cancel_requested = false; // 残留的取消请求不能影响新目标
  }

  /// 句柄到位。与相位**在同一个临界区里**改变，于是
  /// `kRunning` **蕴含** `handle` 非空 —— 由构造保证，不靠时序。
  void begin(std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle) {
    handle = std::move(goal_handle);
    phase = Phase::kRunning;
  }

  void finish() {
    phase = Phase::kIdle;
    cancel_requested = false;
  }
};

} // namespace robot_sdk
