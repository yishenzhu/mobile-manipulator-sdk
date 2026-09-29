#!/usr/bin/env python3
"""导航示例：非阻塞派发 + 边走边看反馈，再一次阻塞调用。

    ros2 run robot_sdk_sim mock_robot_node          # 终端 1
    python3 demo_navigation.py                      # 终端 2
    python3 demo_navigation.py sdk/navigate_to_pose  # 接导航适配器时换个动作名
"""

import sys
import threading

import robot_sdk


def main() -> None:
    config = robot_sdk.RobotConfig(node_name="py_demo_navigation")
    # 动作名可配：mock 提供的是 `navigate_to_pose`，接适配器时要换成 `sdk/navigate_to_pose`
    # —— Nav2 自己占了前者，而**同名但类型不同**的两个 action server 无法共存。
    if len(sys.argv) > 1:
        config.navigation.action_name = sys.argv[1]
    print(f"导航动作名: {config.navigation.action_name}")

    robot = robot_sdk.Robot(config)
    spinner = threading.Thread(target=robot.spin, daemon=True)
    spinner.start()

    if not robot.navigation().server_available(5.0):
        print("❌ 等不到导航服务端 —— 先在另一个终端跑 `ros2 run robot_sdk_sim mock_robot_node`")
        robot.stop()
        spinner.join(timeout=2.0)
        return

    # ── ① 非阻塞派发，边走边看反馈 ──
    goal = robot_sdk.NavigationGoal()
    goal.target = robot_sdk.Pose2D(1.5, 0.5, 0.4)
    goal.avoid_obstacles = True
    goal.timeout_s = 30.0

    print("派发目标 (1.5, 0.5, yaw=0.4)，边走边看反馈...")
    handle = robot.navigation().go_to_async(goal)
    handle.on_feedback(lambda feedback: print(
        f"  反馈: {feedback['current_state']}  剩 {feedback['distance_remaining']:.2f} m"))

    outcome = handle.wait(40.0)
    print(f"通信层: {outcome['code']}")
    # ⚠️ 两层语义：通信层走完了 ≠ 机器人到了；"到没到"在上面那个 dict 里。
    arrived = robot.navigation().current_pose(2.0)
    print(f"现在位姿: x={arrived['x']:.2f} y={arrived['y']:.2f} yaw={arrived['yaw']:.2f}")

    # ── ② 同样的事，一次阻塞调用（目标换到另一侧）──
    print("再来一次，这次用阻塞调用（回原点）...")
    try:
        result = robot.navigation().go_to(robot_sdk.NavigationGoal())
        print(f"结果  : success={result['success']}  末误差 {result['final_distance_error']:.3f}"
              f"  '{result['message']}'")
    except robot_sdk.RobotSdkError as error:
        print(f"❌ {error}")

    robot.stop()
    spinner.join(timeout=2.0)


if __name__ == "__main__":
    main()
