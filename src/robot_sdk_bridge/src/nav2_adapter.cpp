// 把 SDK 的导航动作翻译成 Nav2 的导航动作。
//
//   SDK 客户端 ──robot_sdk_interfaces/NavigateToPose（sdk/navigate_to_pose）──▶ 本节点
//                                                                              │
//              ──nav2_msgs/NavigateToPose（navigate_to_pose）────────────────▶ Nav2
//
// 它是"桥接器"：没有本地状态要推进，只做翻译和转发 —— 所以**不需要定时器**
// （也就用不上 tickAction），终态也由下游（Nav2）决定，本节点不自己判。
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rcl_action/rcl_action.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <robot_sdk_interfaces/action/navigate_to_pose.hpp>

#include "robot_sdk/detail/action_slot.hpp"

namespace robot_sdk::bridge {

class Nav2Adapter : public rclcpp::Node {
 public:
  using SdkAction = robot_sdk_interfaces::action::NavigateToPose;
  using SdkGoalHandle = rclcpp_action::ServerGoalHandle<SdkAction>;
  using Nav2Action = nav2_msgs::action::NavigateToPose;
  using Nav2GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Action>;

  explicit Nav2Adapter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("nav2_adapter", options) {
    // SDK 侧的名字**必须**与 Nav2 自己的 `navigate_to_pose` 不同 ——
    // 同名但类型不同的两个 action server 无法共存。
    sdk_action_name_ =
        declare_parameter<std::string>("sdk_action_name", "sdk/navigate_to_pose");
    nav2_action_name_ = declare_parameter<std::string>("nav2_action_name", "navigate_to_pose");
    default_frame_ = declare_parameter<std::string>("default_frame", "map");
    declare_parameter<std::string>("behavior_tree", "");

    server_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    client_ = rclcpp_action::create_client<Nav2Action>(this, nav2_action_name_, client_group_);
    server_ = rclcpp_action::create_server<SdkAction>(
        this, sdk_action_name_,
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const SdkAction::Goal> goal) {
          return handleGoal(uuid, std::move(goal));
        },
        [this](const std::shared_ptr<SdkGoalHandle> handle) { return handleCancel(handle); },
        [this](const std::shared_ptr<SdkGoalHandle> handle) { handleAccepted(handle); },
        rcl_action_server_get_default_options(), server_group_);
  }

 private:
  // ── 服务端回调 ──

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                         std::shared_ptr<const SdkAction::Goal> goal) {
    bool busy = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      busy = slot_.busy();
      if (!busy) {
        slot_.reserve(); // 占位：下游句柄要等回应回调才到
        last_distance_remaining_ = std::numeric_limits<float>::infinity();
      }
    }
    if (busy) {
      RCLCPP_WARN(get_logger(), "拒绝导航目标：上一个还没结束");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->target_pose.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "目标没写 frame_id，按 '%s' 处理", default_frame_.c_str());
    }
    if (goal->max_speed > 0.0F) {
      // Nav2 的动作没有速度上限字段。
      RCLCPP_WARN(get_logger(), "Nav2 的动作没有 max_speed，%.2f 被忽略", goal->max_speed);
    }
    if (!goal->avoid_obstacles) {
      // Nav2 始终避障，"不避障"表达不了。
      RCLCPP_WARN(get_logger(), "Nav2 始终避障，avoid_obstacles=false 无法兑现");
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<SdkGoalHandle> sdk_handle) {
    if (!sdk_handle->is_active()) { // 问框架在锁外
      return rclcpp_action::CancelResponse::REJECT;
    }

    std::shared_ptr<Nav2GoalHandle> nav2_handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slot_.cancel_requested = true;
      nav2_handle = nav2_handle_;
    }
    // 下游句柄已经到手就立刻转发；还没到的话，等目标回应回调里补发。
    if (nav2_handle) {
      client_->async_cancel_goal(nav2_handle);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<SdkGoalHandle> sdk_handle) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slot_.begin(sdk_handle);
    }

    auto nav2_goal = std::make_shared<Nav2Action::Goal>();
    nav2_goal->pose = sdk_handle->get_goal()->target_pose;
    if (nav2_goal->pose.header.frame_id.empty()) {
      nav2_goal->pose.header.frame_id = default_frame_;
    }
    // 每次发目标时读一次 —— 这样运行时改参数对**下一个**目标就生效，
    // 而不是"改了不生效"的死配置。
    nav2_goal->behavior_tree = get_parameter("behavior_tree").as_string();

    rclcpp_action::Client<Nav2Action>::SendGoalOptions options;
    options.goal_response_callback = [this](const std::shared_ptr<Nav2GoalHandle> handle) {
      onNav2GoalResponse(handle);
    };
    options.feedback_callback = [this](std::shared_ptr<Nav2GoalHandle>,
                                       const std::shared_ptr<const Nav2Action::Feedback> feedback) {
      onNav2Feedback(feedback);
    };
    options.result_callback = [this](const Nav2GoalHandle::WrappedResult& wrapped) {
      onNav2Result(wrapped);
    };

    client_->async_send_goal(*nav2_goal, options);
  }

  // ── 下游（Nav2）客户端的三个回调 ──

  void onNav2GoalResponse(const std::shared_ptr<Nav2GoalHandle> nav2_handle) {
    std::shared_ptr<SdkGoalHandle> sdk_handle;
    bool cancel_late = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sdk_handle = slot_.handle;
      if (nav2_handle) {
        nav2_handle_ = nav2_handle;
        // 取消可能比下游句柄先到 —— 那时转发不了，这里补上。
        cancel_late = slot_.cancel_requested;
      }
    }

    if (!nav2_handle) {
      if (sdk_handle) {
        auto result = std::make_shared<SdkAction::Result>();
        result->success = false;
        result->message = "Nav2 拒绝了导航目标";
        sdk_handle->abort(result);
      }
      std::lock_guard<std::mutex> lock(mutex_);
      slot_.finish();
      return;
    }
    if (cancel_late) {
      client_->async_cancel_goal(nav2_handle);
    }
  }

  void onNav2Feedback(const std::shared_ptr<const Nav2Action::Feedback> feedback) {
    std::shared_ptr<SdkGoalHandle> sdk_handle;
    std::string state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sdk_handle = slot_.handle;
      last_distance_remaining_ = feedback->distance_remaining;
      state = slot_.cancel_requested ? "canceling" : "navigating";
    }
    if (!sdk_handle || !sdk_handle->is_active()) {
      return;
    }

    auto out = std::make_shared<SdkAction::Feedback>();
    out->current_pose = feedback->current_pose;
    out->distance_remaining = feedback->distance_remaining;
    out->estimated_time_remaining =
        static_cast<float>(rclcpp::Duration(feedback->estimated_time_remaining).seconds());
    out->current_state = state;
    sdk_handle->publish_feedback(out);
  }

  void onNav2Result(const Nav2GoalHandle::WrappedResult& wrapped) {
    std::shared_ptr<SdkGoalHandle> sdk_handle;
    float last_distance = 0.0F;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sdk_handle = slot_.handle;
      last_distance = last_distance_remaining_;
      slot_.finish();
      nav2_handle_.reset();
    }
    if (!sdk_handle) {
      return;
    }

    auto result = std::make_shared<SdkAction::Result>();
    // 一条反馈都没收到过时给 inf —— 给 0 会被读成"已经到位"。
    result->final_distance_error = last_distance;
    if (wrapped.result) {
      result->success = wrapped.result->error_code == Nav2Action::Result::NONE;
      result->message = wrapped.result->error_msg;
    }

    // 终态**由下游决定**，本节点不自己判。
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      result->success = false;
      if (result->message.empty()) {
        result->message = "导航被取消";
      }
      sdk_handle->canceled(result);
    } else if (result->success) {
      sdk_handle->succeed(result);
    } else {
      sdk_handle->abort(result);
    }
  }

  std::string sdk_action_name_;
  std::string nav2_action_name_;
  std::string default_frame_;

  rclcpp::CallbackGroup::SharedPtr server_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp_action::Server<SdkAction>::SharedPtr server_;
  rclcpp_action::Client<Nav2Action>::SharedPtr client_;

  /// 保护下面这些成员。回调分落在两条可重入车道上，所以它们真的会并发。
  std::mutex mutex_;
  /// 我方目标的生命周期（三态 + 我方句柄 + 取消信箱）。
  ActionSlot<SdkAction> slot_;
  std::shared_ptr<Nav2GoalHandle> nav2_handle_;
  /// 最后一条反馈里的剩余距离 —— 下游结果里没有这个字段，只能用它。
  float last_distance_remaining_{std::numeric_limits<float>::infinity()};
};

} // namespace robot_sdk::bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<robot_sdk::bridge::Nav2Adapter>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
