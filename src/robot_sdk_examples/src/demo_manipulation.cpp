// 移动操作示例：走到具名位姿 → 动夹爪 → 回 home。
//   ros2 run robot_sdk_sim mock_robot_node                  # 终端 1
//   ros2 run robot_sdk_examples demo_manipulation [动作名]   # 终端 2（接适配器时传 sdk/move_to_joint_positions）
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

  robot_sdk::RobotConfig config{"demo_manipulation"};

  // 动作名可配：mock 提供的是 `move_to_joint_positions`，接适配器时要换。
  if (argc > 1) {
    config.motion.action_name = argv[1];
  }

  // 具名位姿是**部署时**的事：位置向量要和 joint_names **同序同长**，少一个就被 validate 拦下。
  // 夹爪不在这个名单里 —— 它的行程由 URDF 说了算，用 openGripper()/closeGripper() 动。
  config.motion.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                               "wrist_joint", "wrist_pitch_joint", "wrist_roll_joint"};
  config.motion.named_targets["home"] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  config.motion.named_targets["ready"] = {0.5, -0.4, 0.8, 0.2, 0.4, 0.6};

  // 自建模式：SDK 内部自建 MultiThreadedExecutor，`spin()` 转的就是它（它阻塞）。
  robot_sdk::Robot robot(config);
  std::thread spinner([&robot]() { robot.spin(); });

  if (!robot.motion().serverAvailable(5s)) {
    std::printf("❌ 等不到运动服务端 —— 先在另一个终端跑 `ros2 run robot_sdk_sim mock_robot_node`\n");
    robot.stop();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // ── ① 自己拼 goal（唯一能边走边看反馈的写法）──
  std::printf("让手臂走到 'ready' 位姿...\n");
  robot_sdk::MotionGoal goal;
  goal.joint_names = config.motion.joint_names;
  goal.positions = config.motion.named_targets["ready"];
  goal.max_velocity = 0.8;
  goal.timeout_s = 20.0;

  auto handle = robot.motion().moveToJointPositionsAsync(
      goal, [](const robot_sdk::MotionFeedback& feedback) {
        std::printf("  反馈: %.0f%%\n", feedback.completion_percentage);
      });
  auto wrapped = handle->wait(25s);
  if (!wrapped.ok()) {
    std::printf("❌ 通信层就失败了: %s\n", wrapped.error().toString().c_str());
  } else if (wrapped.value().result) {
    // **两层语义**：Result::ok() 只说明"流程走完了"，success 才说明"关节真的到位了"。
    const auto& result = *wrapped.value().result;
    std::printf("手臂  : success=%d  末误差 %.4f  '%s'\n", result.success ? 1 : 0,
                result.final_position_error, result.message.c_str());
  }

  // ── ② 夹爪：张开 / 闭合（阻塞调用）──
  if (auto result = robot.motion().openGripper(); result.ok()) {
    std::printf("夹爪  : 张开 success=%d  '%s'\n", result.value().success ? 1 : 0,
                result.value().message.c_str());
  } else {
    std::printf("❌ 夹爪: %s\n", result.error().toString().c_str());
  }
  std::this_thread::sleep_for(300ms);
  if (auto result = robot.motion().closeGripper(); result.ok()) {
    std::printf("夹爪  : 闭合 success=%d  '%s'\n", result.value().success ? 1 : 0,
                result.value().message.c_str());
  } else {
    std::printf("❌ 夹爪: %s\n", result.error().toString().c_str());
  }

  // ── ③ 具名位姿的捷径：一行回 home；再试个不存在的名字看报错 ──
  if (auto result = robot.motion().home(); result.ok()) {
    std::printf("手臂  : home success=%d\n", result.value().success ? 1 : 0);
  } else {
    std::printf("❌ home: %s\n", result.error().toString().c_str());
  }
  if (auto unknown = robot.motion().moveToNamedTarget("does_not_exist"); !unknown.ok()) {
    std::printf("手臂  : 不存在的名字按预期被拒 -> %s\n", unknown.error().toString().c_str());
  }

  robot.stop();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
