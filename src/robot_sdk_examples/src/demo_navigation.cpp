// 导航示例：先"边跑边看反馈"，再用一次阻塞调用回来。
//   ros2 run robot_sdk_sim mock_robot_node                # 终端 1
//   ros2 run robot_sdk_examples demo_navigation [动作名]   # 终端 2（接 Nav2 适配器时传 sdk/navigate_to_pose）
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "robot_sdk/robot.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  robot_sdk::RobotConfig config{"demo_navigation"};

  // 动作名可配：mock 提供的是 `navigate_to_pose`，接 Nav2 适配器时要换成
  // `sdk/navigate_to_pose` —— Nav2 自己占了前者，而**同名但类型不同**的两个 action
  // server 无法共存。不用改代码就能打这两处，靠的就是这个参数。
  if (argc > 1) {
    config.navigation.action_name = argv[1];
  }
  std::printf("导航动作名: %s\n", config.navigation.action_name.c_str());

  // 自建模式：SDK 内部有个 MultiThreadedExecutor，`spin()` 转的就是它。
  // 它阻塞，所以放后台线程 —— 主线程要用来调阻塞式 API（goTo / wait）。
  robot_sdk::Robot robot(config);
  std::thread spinner([&robot]() { robot.spin(); });

  if (!robot.navigation().serverAvailable(5s)) {
    std::printf("❌ 等不到导航服务端 —— 先在另一个终端跑 `ros2 run robot_sdk_sim mock_robot_node`\n");
    robot.stop();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // ── ① 派发目标 + 边走边看反馈（非阻塞）──
  robot_sdk::NavigationGoal goal;
  goal.target = {2.0, 1.0, 0.0};
  goal.max_speed = 0.5;
  goal.timeout_s = 20.0;
  // avoid_obstacles：默认就是 true。**mock 会真的用它** —— 目标走不到
  // （比如落在墙外）它当场报失败，而不是撞上去。Nav2 那边则是"始终避障"，传 false 也做不到。
  goal.avoid_obstacles = true;

  std::printf("去 (%.1f, %.1f) ...\n", goal.target.x, goal.target.y);

  int feedback_count = 0;
  auto handle =
      robot.navigation().goToAsync(goal, [&feedback_count](const robot_sdk::NavigationFeedback& fb) {
        ++feedback_count;
        if (feedback_count == 1 || feedback_count % 10 == 0) {
          std::printf("  反馈 #%-3d 距目标 %.2f m  预计 %.1f s  状态 %s\n", feedback_count,
                      fb.distance_remaining, fb.estimated_time_remaining, fb.state.c_str());
        }
      });

  auto wrapped = handle->wait(25s);
  if (!wrapped.ok()) {
    std::printf("❌ 通信层就失败了: %s\n", wrapped.error().toString().c_str());
  } else if (wrapped.value().result) {
    const auto& result = *wrapped.value().result;
    // **两层语义**：Result::ok() 只说明"流程走完了"，
    // success 才说明"机器人真的到了"。别把它们混为一谈。
    std::printf("结果  : success=%d  末距 %.3f m  '%s'\n", result.success ? 1 : 0,
                result.final_distance_error, result.message.c_str());
  }

  // ── ② 同样的事，一次阻塞调用 ──
  robot_sdk::NavigationGoal back;
  back.target = {0.0, 0.0, 0.0};
  back.timeout_s = 20.0;

  std::printf("回原点（阻塞调用）...\n");
  if (auto result = robot.navigation().goTo(back); result.ok()) {
    std::printf("结果  : success=%d  末距 %.3f m  '%s'\n", result.value().success ? 1 : 0,
                result.value().final_distance_error, result.value().message.c_str());
  } else {
    std::printf("❌ %s\n", result.error().toString().c_str());
  }

  robot.stop();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
