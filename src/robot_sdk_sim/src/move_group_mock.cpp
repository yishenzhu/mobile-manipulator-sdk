// 假 move_group：**不做任何规划**，只为把 moveit_adapter 的翻译摊开来看。
//
// 它干的全部事情：
//   ① 收到 MoveGroup 目标 → **原样转发**到 `goal_topic`（latch 的，晚到的订阅者也拿得到）
//   ② 等 `delay_s` 之后按 `result` 参数回成功或失败
//   ③ 期间发两条 feedback（PLANNING → MONITOR），好让适配器的反馈翻译有东西可翻
//
// 它**不验**的东西（那些是 MoveIt 的活）：逆解对不对、路径避不避障、时间参数化合不合理、
// max_cartesian_speed 到底有没有生效（那要 LimitMaxCartesianLinkSpeed 适配器参与）。
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rcl_action/rcl_action.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace robot_sdk::sim {

class MoveGroupMock : public rclcpp::Node {
 public:
  using MoveGroupAction = moveit_msgs::action::MoveGroup;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveGroupAction>;

  explicit MoveGroupMock(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("move_group_mock", options) {
    action_name_ = declare_parameter<std::string>("action_name", "/move_action");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "fake_move_group/goal");
    delay_s_ = declare_parameter<double>("delay_s", 0.5);
    // "success" | "fail" —— 让探针能验两条路径（成功、以及失败时适配器怎么报）。
    result_mode_ = declare_parameter<std::string>("result", "success");

    // ⚠️ latch（TRANSIENT_LOCAL）：探针是**先发目标、后订阅**的，
    //    用默认的 VOLATILE 会一条都收不到（这坑今天在 costmap 上踩过）。
    goal_pub_ = create_publisher<MoveGroupAction::Goal>(
        goal_topic_, rclcpp::QoS(1).transient_local());

    server_ = rclcpp_action::create_server<MoveGroupAction>(
        this, action_name_,
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const MoveGroupAction::Goal> goal) {
          return handleGoal(uuid, std::move(goal));
        },
        [this](const std::shared_ptr<GoalHandle> handle) { return handleCancel(handle); },
        [this](const std::shared_ptr<GoalHandle> handle) { handleAccepted(handle); },
        rcl_action_server_get_default_options());

    RCLCPP_INFO(get_logger(), "假 move_group 就绪：动作 '%s'，收到的目标发到 '%s'",
                action_name_.c_str(), goal_topic_.c_str());
  }

 private:
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                         std::shared_ptr<const MoveGroupAction::Goal>) {
    bool busy = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      busy = running_;
      if (!busy) {
        running_ = true;
      }
    }
    if (busy) {
      RCLCPP_WARN(get_logger(), "拒绝：上一条还在跑");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> handle) {
    if (!handle->is_active()) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    timer_.reset(); // 丢掉引用即销毁定时器（TimerBase::cancel 在这个版本里不可用）
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> handle) {
    // ① 把收到的目标原样摊开 —— 探针就靠这个验适配器翻得对不对。
    goal_pub_->publish(*handle->get_goal());

    // ③ 先发一条"在规划"，适配器的反馈翻译才有东西可翻。
    auto feedback = std::make_shared<MoveGroupAction::Feedback>();
    feedback->state = "PLANNING";
    handle->publish_feedback(feedback);

    // ② 等一会儿再回结果 —— 留出时间让探针验"取消"和"超时"。
    timer_ = create_wall_timer(std::chrono::duration<double>(delay_s_), [this, handle]() {
      timer_.reset();

      // ⚠️ 关节点时 rclcpp 会先拆掉 action server，而定时器可能还挂着；此时回结果会抛
      //    "Asked to publish result for goal that does not exist" ⇒ terminate ⇒ SIGABRT。
      //    所以先确认上下文还在（症状很迷惑：进程崩了，测试却记成目标失败）。
      if (!rclcpp::ok()) {
        return;
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
      }

      if (handle->is_canceling()) {
        auto canceled = std::make_shared<MoveGroupAction::Result>();
        handle->canceled(canceled);
        RCLCPP_INFO(get_logger(), "目标被取消");
        return;
      }

      auto second = std::make_shared<MoveGroupAction::Feedback>();
      second->state = "MONITOR";
      handle->publish_feedback(second);

      auto result = std::make_shared<MoveGroupAction::Result>();
      result->planning_time = delay_s_;
      if (result_mode_ == "fail") {
        // 给一个**真实的**错误码，而不是造一个假数字 —— 适配器要把它翻出来。
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
        handle->abort(result);
        RCLCPP_INFO(get_logger(), "按参数回了失败（PLANNING_FAILED）");
      } else {
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        handle->succeed(result);
        RCLCPP_INFO(get_logger(), "按参数回了成功");
      }
    });
  }

  std::string action_name_;
  std::string goal_topic_;
  std::string result_mode_;
  double delay_s_ = 0.5;

  rclcpp_action::Server<MoveGroupAction>::SharedPtr server_;
  rclcpp::Publisher<MoveGroupAction::Goal>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  bool running_ = false;
};

} // namespace robot_sdk::sim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<robot_sdk::sim::MoveGroupMock>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
