#!/usr/bin/env python3
"""验收探针：忙时拒收 + 取消。

本体是 `check()`（不符合预期就 assert），由 test/test_probes.py 调用。
单独跑：

    pytest src/robot_sdk_tests/test/test_probes.py -s -k motion
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_sdk_interfaces.action import MoveToJointPositions

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint",
          "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint"]


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

    def wait(self, future, timeout=30.0):
        # 必须带超时：`spin_until_future_complete` 超时后返回，但 `result()` 是阻塞的，
        # 没完成的话照样一直挂下去，看起来像对端崩了。
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"等 {timeout:.0f} 秒没有回应")
        return future.result()


def check(action_name="/sdk/move_to_joint_positions"):
    """忙时拒收 + 取消。不符合预期就抛 AssertionError / TimeoutError。"""
    rclpy.init()
    try:
        print(f"动作名: {action_name}")
        probe = Probe(action_name)
        assert probe.client.wait_for_server(timeout_sec=10.0), \
            f"等不到动作服务端 {action_name}"

        # ── ① 忙时拒收 ──
        # 目标要**慢**（这样第二个目标发来时它还在跑），所以走一段长的**手臂**动作。
        # ⚠️ 夹爪只能给 0～0.03（URDF 说它是 prismatic）—— 给它 3.0 会被机器人当场拒，
        #    那是**对的**：限位归执行侧管。
        print("① 发一个慢目标（shoulder_lift 走 1.5 rad）...")
        first = probe.wait(probe.client.send_goal_async(
            probe.goal([0.5, 1.5, 0.2, 0.0, 0.0, 0.0, 0.01], velocity=0.5, acceleration=1.0)))
        assert first.accepted, "第一个目标该被接受"

        print("② 在它跑的时候再发一个 —— 应该被拒")
        second = probe.wait(probe.client.send_goal_async(
            probe.goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity=0.5, acceleration=1.0)))
        assert not second.accepted, "忙时不该接受第二个目标"

        # ── ③ 取消 ──
        print("③ 取消第一个目标")
        cancel_response = probe.wait(first.cancel_goal_async())
        assert cancel_response.goals_canceling, "取消该被受理"

        wrapped = probe.wait(first.get_result_async()).result
        print(f"   终态 success={wrapped.success}  message='{wrapped.message}'")
        assert not wrapped.success, "取消之后不该报成功"
        print("✅ 忙时拒收 + 取消：都对")
    finally:
        rclpy.shutdown()
