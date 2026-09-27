// 关节运动示例：走一遍 SDK 运动 API 的两种用法（边走边看反馈 / 一次阻塞调用）。
//
//   # 终端 1
//   ros2 run robot_sdk_sim mock_robot_node
//   # 终端 2
//   ros2 run robot_sdk_examples demo_motion
//
// 接上运动适配器时，动作名要换（同 demo_navigation）：
//   ros2 run robot_sdk_examples demo_motion sdk/move_to_joint_positions
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "robot_sdk/robot.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  robot_sdk::RobotConfig config{"demo_motion"};

  // 动作名可配：mock 机器人提供的名字是 `move_to_joint_positions`；
  // 接上运动适配器时必须换成 `sdk/move_to_joint_positions`。
  if (argc > 1) {
    config.motion.action_name = argv[1];
  }
  std::printf("运动动作名: %s\n", config.motion.action_name.c_str());

  // 自建模式：SDK 内部有个 MultiThreadedExecutor，`spin()` 转的就是它。
  // 它阻塞，所以放后台线程 —— 主线程要用来调阻塞式 API。
  robot_sdk::Robot robot(config);
  std::thread spinner([&robot]() { robot.spin(); });

  if (!robot.motion().serverAvailable(5s)) {
    std::printf("❌ 等不到运动服务端 —— 先在另一个终端跑 `ros2 run robot_sdk_sim mock_robot_node`\n");
    robot.stop();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // ── ① 关节名问机器人要，别在客户代码里写死 ──
  auto joints_msg = robot.status().waitForJointStates(5s);
  if (!joints_msg.ok()) {
    std::printf("❌ 拿不到关节状态: %s\n", joints_msg.error().toString().c_str());
    robot.stop();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }
  const std::vector<std::string> joints = joints_msg.value()->name;
  std::printf("机器人有 %zu 个关节:", joints.size());
  for (const auto& name : joints) {
    std::printf(" %s", name.c_str());
  }
  std::printf("\n");

  // ── ② 派发目标 + 边走边看反馈（非阻塞）──
  // 目标里**只写要动的那几个关节** —— 没点名的保持原位。
  robot_sdk::MotionGoal goal;
  goal.joint_names = {joints.front()};
  goal.positions = {0.3};
  goal.max_velocity = 0.5;    // 每个关节各自的速度上限；<= 0 表示由后端决定
  goal.timeout_s = 20.0;

  std::printf("让 '%s' 转到 0.3 ...\n", goal.joint_names.front().c_str());
  int feedback_count = 0;
  auto handle = robot.motion().moveToJointPositionsAsync(
      goal, [&feedback_count](const robot_sdk::MotionFeedback& fb) {
        // 完成度是**百分数**（0 → 100），与 mock 的语义一致。
        if (++feedback_count == 1 || feedback_count % 10 == 0) {
          std::printf("  反馈 #%-3d 完成度 %.1f%%\n", feedback_count, fb.completion_percentage);
        }
      });

  auto wrapped = handle->wait(25s);
  if (!wrapped.ok()) {
    std::printf("❌ 通信层就失败了: %s\n", wrapped.error().toString().c_str());
  } else if (wrapped.value().result) {
    const auto& result = *wrapped.value().result;
    // **两层语义**：Result::ok() 只说明"流程走完了"，success 才说明"关节真的到位了"。
    std::printf("结果  : success=%d  末误差 %.4f  '%s'\n", result.success ? 1 : 0,
                result.final_position_error, result.message.c_str());
  }

  // ── ③ 同样的事，一次阻塞调用 —— 张开夹爪 ──
  std::printf("张开夹爪（阻塞调用）...\n");
  if (auto result = robot.motion().openGripper(); result.ok()) {
    std::printf("结果  : success=%d  末误差 %.4f  '%s'\n", result.value().success ? 1 : 0,
                result.value().final_position_error, result.value().message.c_str());
  } else {
    std::printf("❌ %s\n", result.error().toString().c_str());
  }

  robot.stop();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
