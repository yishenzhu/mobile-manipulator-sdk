#pragma once

#include <memory>
#include <mutex>

#include "robot_sdk/detail/action_slot.hpp"

namespace robot_sdk::sim {

/// `advance` 这一拍的产出。两个字段都可空，且**至多填一个**：
/// 填了 `result` 就表示本目标结束 —— 骨架按"客户是否请求过取消"回 `canceled` 或 `succeed`。
template <typename Action>
struct TickOutput {
  std::shared_ptr<typename Action::Result> result;
  std::shared_ptr<typename Action::Feedback> feedback;
};

/// 动作服务端的**每拍骨架** —— 给"自己按时间推进"的服务端用（模拟器、本体驱动那类）；
/// 转发的桥接器用不上（没有本地状态要推进）。骨架管加锁 / 空转 / 终态回调 / 反馈发布，
/// 并保证框架调用全在锁外；`advance` 在锁内被调用，且只在正常推进时（取消由骨架处理）。
template <typename Action, typename Advance>
void tickAction(std::mutex &mutex, robot_sdk::ActionSlot<Action> &slot, Advance &&advance) {
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> handle;
  TickOutput<Action> output;
  bool canceled = false;

  {
    // ── 锁内：推进数据 + 判定终态（不碰框架）──
    std::lock_guard<std::mutex> lock(mutex);
    if (!slot.running()) {
      return; // 空转 —— 也顺带保证下面的 handle 非空
    }
    handle = slot.handle;

    if (slot.cancel_requested) {
      output.result = std::make_shared<typename Action::Result>();
      canceled = true;
    } else {
      output = advance(lock);
    }
    if (output.result) {
      slot.finish();
    }
  } // ← 锁在这里放掉：下面全是框架调用（I/O）

  if (output.result) {
    if (canceled) {
      handle->canceled(output.result);
    } else {
      handle->succeed(output.result);
    }
    return;
  }
  if (output.feedback) {
    handle->publish_feedback(output.feedback);
  }
}

} // namespace robot_sdk::sim
