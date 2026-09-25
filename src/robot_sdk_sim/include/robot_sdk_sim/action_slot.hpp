#pragma once

#include <memory>
#include <mutex>
#include <utility>

#include <rclcpp_action/rclcpp_action.hpp>

namespace robot_sdk::sim {

/// 一个动作服务端的**槽位**。本模拟器同时只跑一个同类目标，所以每个 action
/// 只要一个槽位 —— 它把"这个 action 现在处于什么状态"收成**一个值**。
///
/// **为什么不是三个 bool**（`reserved` / `active` / `cancel_requested`）：
/// 三个 bool 有 8 种组合，只有 4 种合法 —— 而 `reserved && active` 正是
/// 非法组合之一，上一轮的悬空句柄 bug 就住在那里。改成三态枚举后，
/// 非法组合**根本无法表达**。
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

  /// **一位信箱**，不是状态：取消回调（框架线程）投递，控制回路取走。
  /// 它**任何相位都可能到达**（连 `kIdle` 都可能 —— 取消回调里 `is_active()`
  /// 与这里加锁之间目标可能已结束，见 学习笔记.md §7.8），所以不能并进 `Phase`。
  bool cancel_requested{false};

  /// 反馈降频用：每 N 个控制周期发一条反馈。
  int feedback_divider{0};

  bool busy() const { return phase != Phase::kIdle; }
  bool running() const { return phase == Phase::kRunning; }

  /// 占位：挡住并发到达的第二个目标。**不置 `kRunning`** —— 句柄还没到。
  void reserve() {
    phase = Phase::kReserved;
    cancel_requested = false; // 残留的取消请求不能影响新目标（笔记 §7.8）
    feedback_divider = 0;
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

/// 动作服务端的**每拍骨架**。
///
/// 两个 `*Tick()` 本来只差中间那段"推进"，其余（局部变量、加锁、空转、
/// 取消分支、终态回调、反馈发布）**逐字相同** —— 那就只写一遍。
///
/// `advance` 在**锁内**被调用，所以它拿得到凭证 `lock`（同 `makeStatus`）。
/// 它只需回答"这一拍做了什么"：**返回非空 result 即表示完成** ——
/// 用"有没有结果"代替一个 `bool done` 出参，少一个能写错的开关。
///
/// 框架调用（`succeed` / `canceled` / `publish_feedback`）全在**锁外**。
template <typename Action, typename Advance>
void tickAction(std::mutex &mutex, ActionSlot<Action> &slot, Advance &&advance) {
  using Result = typename Action::Result;
  using Feedback = typename Action::Feedback;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> handle;
  std::shared_ptr<Result> result;
  std::shared_ptr<Feedback> feedback;
  bool canceled = false;

  {
    // ── 锁内：推进数据 + 判定终态（不碰框架）──
    std::lock_guard<std::mutex> lock(mutex);
    if (!slot.running()) {
      return; // 空转 —— 也顺带保证下面的 handle 非空
    }
    handle = slot.handle;

    if (slot.cancel_requested) {
      result = std::make_shared<Result>();
      result->message = "canceled by client";
      canceled = true;
    } else {
      advance(lock, result, feedback);
    }
    if (result) {
      slot.finish();
    }
  } // ← 锁在这里放掉：下面全是框架调用（I/O）

  if (result) {
    if (canceled) {
      handle->canceled(result);
    } else {
      handle->succeed(result);
    }
    return;
  }
  if (feedback) {
    handle->publish_feedback(feedback);
  }
}

} // namespace robot_sdk::sim
