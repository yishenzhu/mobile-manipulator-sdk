#!/usr/bin/env python3
"""验收探针：连续关节的折角。

从"当前位置"发到"当前位置 + 6.7 rad"：
折角对了 → 只走 6.7 - 2π = 0.417 rad；没折角 → 老实走 6.7 rad。
两者一个 0.8 秒一个 13 秒，谁也冒充不了谁。

    ros2 run robot_sdk_tests probe_wrap                          # 打 mock
    ros2 run robot_sdk_tests probe_wrap /sdk/move_to_joint_positions   # 打适配器

前提：目标那一侧得把 `shoulder_pan_joint` 声明成连续关节，否则走的就是 6.7。
mock 用 `--ros-args -p motion.continuous_joints:=[shoulder_pan_joint]`，
适配器用 `--ros-args -p continuous_joints:=[shoulder_pan_joint]`。
"""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_sdk_interfaces.action import MoveToJointPositions

JOINT = "shoulder_pan_joint"
DELTA = 6.7      # 目标 = 当前 + 6.7 rad（绕一圈还多 0.417）


class Probe(Node):
    def __init__(self, action_name):
        super().__init__("probe_wrap")
        self.positions = []
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.client = ActionClient(self, MoveToJointPositions, action_name)

    def on_joint_state(self, msg):
        if JOINT in msg.name:
            self.positions.append((time.monotonic(), msg.position[msg.name.index(JOINT)]))

    def wait(self, future):
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def current(self):
        while len(self.positions) < 2:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.positions[-1][1]

    def run_once(self, target):
        time.sleep(0.3)
        before = len(self.positions)
        goal = MoveToJointPositions.Goal()
        goal.target.name = [JOINT]
        goal.target.position = [target]
        goal.max_velocity = 0.5
        goal.max_acceleration = 0.0       # 不限，让耗时只反映走了多长
        handle = self.wait(self.client.send_goal_async(goal))
        if not handle.accepted:
            return None
        self.wait(handle.get_result_async())
        time.sleep(0.4)

        window = self.positions[before:]
        changes = [window[i][0] for i in range(1, len(window))
                   if abs(window[i][1] - window[i - 1][1]) > 1e-6]
        return (changes[-1] - changes[0] if changes else 0.0), window[-1][1]


def main():
    rclpy.init()
    action_name = sys.argv[1] if len(sys.argv) > 1 else "/move_to_joint_positions"
    node = Probe(action_name)
    if not node.client.wait_for_server(timeout_sec=10.0):
        print(f"❌ 等不到动作服务端 {action_name}")
        return 1

    start = node.current()
    target = start + DELTA
    print(f"  起点 {start:.3f} rad，目标 {target:.3f} rad（差 {DELTA}）")

    out = node.run_once(target)
    if out is None:
        print("  ❌ 目标被拒")
        return 1
    duration, end = out
    print(f"  实际停在 {end:.3f} rad  →  走了 {end - start:+.3f} rad")
    print(f"  运动耗时 {duration:.2f} 秒")

    travelled = abs(end - start)
    wrapped = abs(travelled - (DELTA - 6.283185307179586)) < 0.05
    print("  → " + ("✅ 折角生效（走了 0.417，没绕一圈）" if wrapped
                    else "❌ 没折角（老实走了 6.7）"))

    node.destroy_node()
    rclpy.shutdown()
    return 0 if wrapped else 1


def cli():
    """console_scripts 的入口 —— 它**不看** main() 的返回值，得自己转成退出码。"""
    sys.exit(main())


if __name__ == "__main__":
    sys.exit(main())
