#!/usr/bin/env python3
"""验收探针：max_acceleration 真的生效了吗。

同样走 0.4 rad、速度上限都是 0.5 m/s，**只改加速度**：

    不限   → 理论 0.4 / 0.5 = 0.80 秒
    a=0.2  → 理论 2√(0.4/0.2) = 2.83 秒（三角形，达不到顶速）

时长**从 /joint_states 上量**：第一个"位置开始变"的样本 → 最后一个"还在变"的样本。
**不要**用 `ros2 action send_goal` 掐表 —— 那个客户端自带 1.4~1.9 秒启动开销，还抖 0.5 秒。

    ros2 run robot_sdk_tests probe_accel                          # 打 mock
    ros2 run robot_sdk_tests probe_accel /sdk/move_to_joint_positions   # 打适配器
"""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_sdk_interfaces.action import MoveToJointPositions

JOINT = "shoulder_pan_joint"
TARGET = 0.4          # 每次走同样的距离
VELOCITY = 0.5


class Measure(Node):
    def __init__(self, action_name):
        super().__init__("probe_accel")
        self.samples = []          # (墙钟, 角度)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.client = ActionClient(self, MoveToJointPositions, action_name)

    def on_joint_state(self, msg):
        if JOINT in msg.name:
            self.samples.append((time.monotonic(), msg.position[msg.name.index(JOINT)]))

    def wait(self, future):
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def move(self, target, accel):
        goal = MoveToJointPositions.Goal()
        goal.target.name = [JOINT]
        goal.target.position = [target]
        goal.max_velocity = VELOCITY
        goal.max_acceleration = accel
        handle = self.wait(self.client.send_goal_async(goal))
        if not handle.accepted:
            return None
        self.wait(handle.get_result_async())
        return handle

    def measure(self, target, accel):
        """跑一次，返回 (运动时长, 首末位置)。"""
        time.sleep(0.3)                       # 让数据流先稳
        before = len(self.samples)
        self.move(target, accel)
        time.sleep(0.4)                       # 等收尾的样本

        window = self.samples[before:]
        if len(window) < 3:
            return None
        changes = [window[i][0]
                   for i in range(1, len(window))
                   if abs(window[i][1] - window[i - 1][1]) > 1e-6]
        if not changes:
            return None
        return changes[-1] - changes[0], window[0][1], window[-1][1]


def main():
    rclpy.init()
    action_name = sys.argv[1] if len(sys.argv) > 1 else "/move_to_joint_positions"
    node = Measure(action_name)
    if not node.client.wait_for_server(timeout_sec=10.0):
        print(f"❌ 等不到动作服务端 {action_name}")
        return 1

    print(f"动作名 {action_name}；每次走 {TARGET} rad，速度上限 {VELOCITY}")
    print(f"理论：不限加速度 ≈ {TARGET / VELOCITY:.2f} 秒；"
          f"a=0.2 ≈ {2 * (TARGET / 0.2) ** 0.5:.2f} 秒")
    print()

    results = {}
    for accel, target in ((100.0, TARGET), (0.2, 0.0), (100.0, TARGET), (0.2, 0.0)):
        out = node.measure(target, accel)
        if out is None:
            print(f"  a={accel:<6} → 没测到")
            continue
        duration, first, last = out
        results.setdefault(accel, []).append(duration)
        print(f"  a={accel:<6} → 运动 {duration:.3f} 秒   ({first:.3f} → {last:.3f})")

    node.destroy_node()
    rclpy.shutdown()

    if not (results.get(100.0) and results.get(0.2)):
        print("❌ 有两档没测到，判不了")
        return 1

    fast = min(results[100.0])
    slow = min(results[0.2])
    ratio = slow / fast
    print(f"\n  不限加速度 {fast:.3f} 秒  vs  a=0.2 {slow:.3f} 秒  → 慢 {ratio:.2f} 倍")

    # 判据：加速度小的一大截慢（理论 3.5 倍，留点余量）。
    ok = ratio > 2.0
    print("✅ 验收通过" if ok else "❌ 验收未通过")
    return 0 if ok else 1


def cli():
    """console_scripts 的入口 —— 它**不看** main() 的返回值，得自己转成退出码。"""
    sys.exit(main())


if __name__ == "__main__":
    sys.exit(main())
