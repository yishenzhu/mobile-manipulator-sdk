#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_sdk_interfaces/action/navigate_to_pose.hpp>

#include "robot_sdk/action_handle.hpp"
#include "robot_sdk/config.hpp"
#include "robot_sdk/result.hpp"
#include "robot_sdk/transform.hpp"
#include "robot_sdk/types.hpp"

namespace robot_sdk {

/// 高层导航目标。
/// frame 留空、timeout_s 传 0 时都会落到配置里的默认值 ——
/// 于是最常见的"开到 (x, y, yaw)"就是一行。
struct NavigationGoal {
  Pose2D target{};

  /// 目标位姿表达在哪个坐标系。空 = 用 `NavigationConfig::global_frame`。
  std::string frame{};

  /// 速度上限。**<= 0 表示"由后端自己决定"**（哨兵值，与 timeout_s 同一约定）。
  double max_speed{0.0};

  /// 是否避障。true = 走不到就别走（mock 会预检目标可达性，挡着就报失败，但它不绕路）；
  /// false = 不管直冲（Nav2 适配器**始终避障**，兑现不了 false，会打 WARN）。
  bool avoid_obstacles{true};

  /// 超时秒数。**<= 0 = 用 `NavigationConfig::default_timeout_s`**。
  double timeout_s{0.0};
};

/// 导航的终态。注意它与 `Result` 是**两层**错误语义：
/// `Result.ok()==true` 只说明"流程走完了"，`success` 才说明"机器人真的到了"。
struct NavigationResult {
  bool success{false};
  double final_distance_error{0.0};
  std::string message{};
};

/// 导航过程中的进度。
struct NavigationFeedback {
  Pose2D current_pose{};
  double distance_remaining{0.0};
  double estimated_time_remaining{0.0};
  std::string state{};
};

/// "去某个地方"这一层抽象的导航能力。实现只跟 **SDK 自己的导航 action** 打交道
/// （由适配器转到 Nav2 上），所以客户不依赖 Nav2 的消息类型，也不关心导航栈换不换。
class NavigationAPI {
 public:
  using Action = robot_sdk_interfaces::action::NavigateToPose;
  using FeedbackCallback = std::function<void(const NavigationFeedback&)>;

  NavigationAPI(const rclcpp::Node::SharedPtr& node,
                const rclcpp::CallbackGroup::SharedPtr& callback_group,
                std::shared_ptr<TransformManager> transform, const NavigationConfig& config);

  bool serverAvailable(std::chrono::duration<double> timeout);

  /// 派发目标，**等到服务端"接受"再返回**（对齐 nav2）—— 所以取消不会有"还没有 goal handle"的空窗。
  /// ⚠️ 可能返回**空指针**：超时未被接受、或服务端拒收（超时上限 `server_wait_timeout_s`）。
  /// 拿到后进度可以用 `on_feedback` 挂，也可以稍后在句柄上调 `onFeedback()`。
  std::shared_ptr<ActionHandle<Action>> goToAsync(const NavigationGoal& goal,
                                                  FeedbackCallback on_feedback = nullptr);

  /// 派发目标并阻塞到它完成或超时。
  /// 返回的 `Result.ok()==true` 时里面**仍有** success 标志：
  /// "传输失败"和"机器人没走到"是两回事。
  Result<NavigationResult> goTo(const NavigationGoal& goal);

  /// 机器人在配置的全局坐标系下的当前平面位姿。
  Result<Pose2D> currentPose(
      std::chrono::duration<double> timeout = std::chrono::duration<double>(0.1)) const;

  /// 取消本 API 派发过的所有目标。
  void cancelAll();

 private:
  Action::Goal makeGoalMessage(const NavigationGoal& goal) const;
  static NavigationResult toResult(const typename ActionHandle<Action>::WrappedResult& wrapped);

  rclcpp::Node::SharedPtr node_;
  NavigationConfig config_;
  std::shared_ptr<TransformManager> transform_;
  std::shared_ptr<rclcpp_action::Client<Action>> client_;

  /// 用 weak_ptr 记录活跃句柄：句柄的生命周期归客户，这里只是"登记在册"，
  /// 好让 cancelAll() 找得到它们。cancelAll() 会顺手清掉已过期的登记。
  mutable std::mutex handles_mutex_;
  std::vector<std::weak_ptr<ActionHandle<Action>>> active_handles_;
};

}  // namespace robot_sdk
