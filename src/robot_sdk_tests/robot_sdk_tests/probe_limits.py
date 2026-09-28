#!/usr/bin/env python3
"""验收探针：超限的关节目标该被**当场拒绝**。

限位是**机器人的属性**（写在它的 URDF 里）—— 所以"能不能到"这个判断归**执行侧**，
SDK 不替它做（SDK 是硬件无关的，两处各存一份规则迟早漂移）。

打的是 mock 机器人。`shoulder_lift_joint` 在 `robot_description` 的 URDF 里是 ±1.8：

    超限 5.0  →  目标被拒（不是转过去）
    合规 1.5  →  正常走到

本体是 `check()`（不符合预期就 assert），由 test/test_probes.py 调用。单独跑：

    pytest src/robot_sdk_tests/test/test_probes.py -s -k limits
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_sdk_interfaces.action import MoveToJointPositions

JOINT = "shoulder_lift_joint"
LIMIT = 1.8      # 和 robot_description 里那份 URDF 对齐（改 URDF 就得改这里）
INSIDE = 1.5
OUTSIDE = 5.0


class Probe(Node):
    def __init__(self, action_name):
        super().__init__("probe_limits")
        self.client = ActionClient(self, MoveToJointPositions, action_name)

    def wait(self, future, timeout=30.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"等 {timeout:.0f} 秒没有回应")
        return future.result()

    def go(self, position):
        goal = MoveToJointPositions.Goal()
        goal.target.name = [JOINT]
        goal.target.position = [position]
        goal.max_velocity = 0.5
        goal.max_acceleration = 0.0
        return self.wait(self.client.send_goal_async(goal))


def check(action_name="/move_to_joint_positions"):
    """超限该拒、合规该过。不符合预期就 assert。"""
    rclpy.init()
    try:
        probe = Probe(action_name)
        assert probe.client.wait_for_server(timeout_sec=10.0), \
            f"等不到动作服务端 {action_name}"

        handle = probe.go(OUTSIDE)
        print(f"  超限 {OUTSIDE}（{JOINT} 的限位是 ±{LIMIT}）→ accepted={handle.accepted}")
        assert not handle.accepted, f"{OUTSIDE} 超出 ±{LIMIT}，目标不该被接受"

        handle = probe.go(INSIDE)
        assert handle.accepted, f"{INSIDE} 在限位内，目标该被接受"
        result = probe.wait(handle.get_result_async()).result
        print(f"  合规 {INSIDE} → success={result.success}  '{result.message}'")
        assert result.success, "限位内的目标该能走到"
        print("✅ 超限被拒、合规放行")
    finally:
        rclpy.shutdown()
