#!/usr/bin/env python3
"""验收探针：忙时拒收 + 取消。

    ros2 run robot_sdk_tests probe_motion            # 打适配器（默认）
    ros2 run robot_sdk_tests probe_motion move_to_joint_positions   # 打 mock
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_sdk_interfaces.action import MoveToJointPositions

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint",
          "gripper_joint"]


class Probe(Node):
    def __init__(self, action_name="/sdk/move_to_joint_positions"):
        super().__init__("motion_probe")
        self.client = ActionClient(self, MoveToJointPositions, action_name)

    def goal(self, positions, velocity=0.5, acceleration=1.0):
        msg = MoveToJointPositions.Goal()
        msg.target.name = JOINTS
        msg.target.position = positions
        msg.max_velocity = velocity
        msg.max_acceleration = acceleration
        return msg

    def wait(self, future):
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    action_name = sys.argv[1] if len(sys.argv) > 1 else "/sdk/move_to_joint_positions"
    print(f"动作名: {action_name}")
    probe = Probe(action_name)
    if not probe.client.wait_for_server(timeout_sec=10.0):
        print("❌ 等不到动作服务端")
        return 1

    # ── ① 忙时拒收 ──
    print("① 发一个慢目标（joint5 走 2.0 rad）...")
    first = probe.wait(probe.client.send_goal_async(
        probe.goal([0.5, -0.3, 0.2, 0.0, 3.0], velocity=0.5, acceleration=1.0)))
    print(f"   第一个目标被接受: {first.accepted}")

    print("② 在它跑的时候再发一个 —— 应该被拒")
    second = probe.wait(probe.client.send_goal_async(
        probe.goal([0.0, 0.0, 0.0, 0.0, 0.0], velocity=0.5, acceleration=1.0)))
    print(f"   第二个目标被接受: {second.accepted}   {'✅ 拒了' if not second.accepted else '❌ 不该接受'}")

    # ── ③ 取消 ──
    print("③ 取消第一个目标")
    cancel_response = probe.wait(first.cancel_goal_async())
    print(f"   取消返回码数量: {len(cancel_response.goals_canceling)}")

    result = probe.wait(first.get_result_async())
    wrapped = result.result
    print(f"   终态 success={wrapped.success}  message='{wrapped.message}'")
    ok = (not second.accepted) and (not wrapped.success)
    print("✅ 验收通过" if ok else "❌ 验收未通过")

    probe.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


def cli():
    """console_scripts 的入口 —— 它**不看** main() 的返回值，得自己转成退出码。"""
    sys.exit(main())


if __name__ == "__main__":
    sys.exit(main())
