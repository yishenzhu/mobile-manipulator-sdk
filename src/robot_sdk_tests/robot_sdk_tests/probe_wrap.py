#!/usr/bin/env python3
"""验收探针：连续关节的折角。

从"当前位置"发到"当前位置 + 6.7 rad"：
折角对了 → 只走 6.7 - 2π = 0.417 rad；没折角 → 老实走 6.7 rad。
两者一个 0.8 秒一个 13 秒，谁也冒充不了谁。

前提：目标那一侧得把 `shoulder_pan_joint` 声明成连续关节，否则走的就是 6.7 ——
`check()` 会**如实报失败**。test/conftest.py 的 fixture 起 mock 时就带上了 `-p`。

本体是 `check()`（不符合预期就 assert），由 test/test_probes.py 调用。单独跑：

    pytest src/robot_sdk_tests/test/test_probes.py -s -k wrap
"""

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_sdk_interfaces.action import MoveToJointPositions

JOINT = "shoulder_pan_joint"
DELTA = 6.7      # 目标 = 当前 + 6.7 rad（绕一圈还多 0.417）
WRAPPED = DELTA - 6.283185307179586    # 折角之后该走这么多


class Probe(Node):
    def __init__(self, action_name):
        super().__init__("probe_wrap")
        self.positions = []
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.client = ActionClient(self, MoveToJointPositions, action_name)

    def on_joint_state(self, msg):
        if JOINT in msg.name:
            self.positions.append((time.monotonic(), msg.position[msg.name.index(JOINT)]))

    def wait(self, future, timeout=30.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"等 {timeout:.0f} 秒没有回应")
        return future.result()

    def current(self, timeout=30.0):
        """等第一份关节状态 —— 也**要有上限**，否则对端不发话题就死循环。"""
        deadline = time.monotonic() + timeout
        while len(self.positions) < 2:
            if time.monotonic() > deadline:
                raise TimeoutError(f"等 {timeout:.0f} 秒没有收到 /joint_states")
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
        assert handle.accepted, "目标该被接受"
        self.wait(handle.get_result_async())
        time.sleep(0.4)

        window = self.positions[before:]
        changes = [window[i][0] for i in range(1, len(window))
                   if abs(window[i][1] - window[i - 1][1]) > 1e-6]
        return (changes[-1] - changes[0] if changes else 0.0), window[-1][1]


def check(action_name="/move_to_joint_positions"):
    """走最短的一边。不符合预期就 assert。"""
    rclpy.init()
    try:
        node = Probe(action_name)
        assert node.client.wait_for_server(timeout_sec=10.0), \
            f"等不到动作服务端 {action_name}"

        start = node.current()
        print(f"  起点 {start:.3f} rad，目标 {start + DELTA:.3f} rad（差 {DELTA}）")

        duration, end = node.run_once(start + DELTA)
        travelled = abs(end - start)
        print(f"  实际停在 {end:.3f} rad  →  走了 {travelled:.3f} rad，耗时 {duration:.2f} 秒")

        assert abs(travelled - WRAPPED) < 0.05, \
            f"该走 {WRAPPED:.3f} rad（最短的一边），实际走了 {travelled:.3f}"
        print("✅ 折角生效：走的是最短的一边")
    finally:
        rclpy.shutdown()
