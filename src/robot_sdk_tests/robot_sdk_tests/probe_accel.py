#!/usr/bin/env python3
"""验收探针：max_acceleration 真的生效了吗。

同样走 0.4 rad、速度上限都是 0.5 m/s，**只改加速度**：

    不限   → 理论 0.4 / 0.5 = 0.80 秒
    a=0.2  → 理论 2√(0.4/0.2) = 2.83 秒（三角形，达不到顶速）

时长**从 /joint_states 上量**：第一个"位置开始变"的样本 → 最后一个"还在变"的样本。
**不要**用 `ros2 action send_goal` 掐表 —— 那个客户端自带 1.4~1.9 秒启动开销，还抖 0.5 秒。

本体是 `check()`（不符合预期就 assert），由 test/test_probes.py 调用。单独跑：

    pytest src/robot_sdk_tests/test/test_probes.py -s -k accel
"""

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_sdk_interfaces.action import MoveToJointPositions

JOINT = "shoulder_pan_joint"
START = -0.2          # 每条测量都从这儿出发 / 回到这儿 → 每条都恰好 0.4 rad
TARGET = 0.2
VELOCITY = 0.5
MIN_RATIO = 2.0       # 理论 3.5 倍，留点余量


class Measure(Node):
    def __init__(self, action_name):
        super().__init__("probe_accel")
        self.samples = []          # (墙钟, 角度)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.client = ActionClient(self, MoveToJointPositions, action_name)

    def on_joint_state(self, msg):
        if JOINT in msg.name:
            self.samples.append((time.monotonic(), msg.position[msg.name.index(JOINT)]))

    def wait(self, future, timeout=30.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"等 {timeout:.0f} 秒没有回应")
        return future.result()

    def move(self, target, accel):
        goal = MoveToJointPositions.Goal()
        goal.target.name = [JOINT]
        goal.target.position = [target]
        goal.max_velocity = VELOCITY
        goal.max_acceleration = accel
        handle = self.wait(self.client.send_goal_async(goal))
        assert handle.accepted, "目标该被接受"
        self.wait(handle.get_result_async())

    def measure(self, target, accel):
        """跑一次，返回运动时长；量不到就抛。"""
        # ⚠️ 先等订阅**真的**收到样本：否则 /joint_states 的发现还没完成，
        #    整个窗口一条都没有，会被误报成"没量到运动"（实测踩过一次）。
        deadline = time.monotonic() + 10.0
        while not self.samples and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        assert self.samples, "10 秒内一条 /joint_states 都没收到 —— 订阅或发现有问题"

        time.sleep(0.3)                       # 让数据流先稳
        before = len(self.samples)
        self.move(target, accel)
        time.sleep(0.4)                       # 等收尾的样本

        window = self.samples[before:]
        changes = [window[i][0]
                   for i in range(1, len(window))
                   if abs(window[i][1] - window[i - 1][1]) > 1e-6]
        assert changes, f"a={accel} 这一档没量到运动（/joint_states 没动？）"
        return changes[-1] - changes[0]


def check(action_name="/move_to_joint_positions"):
    """加速度小的必须明显慢。不符合预期就 assert。"""
    rclpy.init()
    try:
        node = Measure(action_name)
        assert node.client.wait_for_server(timeout_sec=10.0), \
            f"等不到动作服务端 {action_name}"

        print(f"动作名 {action_name}；每次走 {TARGET - START} rad，速度上限 {VELOCITY}")

        # 先回到已知起点 —— 关节停在哪儿取决于**上一个跑过的东西**（单跑时是 mock 的
        # 初值，放进 pytest 就是上一个用例留下的位置）。不归位的话，第一条测量的
        # 距离是未知的，挪得太小还会量不到。
        node.move(START, 100.0)
        time.sleep(0.3)

        durations = {}
        for accel, target in ((100.0, TARGET), (0.2, START), (100.0, TARGET), (0.2, START)):
            duration = node.measure(target, accel)
            durations.setdefault(accel, []).append(duration)
            print(f"  a={accel:<6} → {START if target == TARGET else TARGET:+.1f} → {target:+.1f}"
                  f"  运动 {duration:.3f} 秒")

        fast = min(durations[100.0])
        slow = min(durations[0.2])
        print(f"  不限加速度 {fast:.3f} 秒  vs  a=0.2 {slow:.3f} 秒  → 慢 {slow / fast:.2f} 倍")

        assert slow / fast > MIN_RATIO, \
            f"a=0.2 只慢 {slow / fast:.2f} 倍（该 > {MIN_RATIO}）—— 加速度没生效？"
        print("✅ max_acceleration 真的在起作用")
    finally:
        rclpy.shutdown()
