#!/usr/bin/env python3
"""验收探针：笛卡尔那条路 —— 适配器把 SDK 的「位姿」翻成 MoveGroup 的「约束」没有。

打的是「`move_group_mock` + `moveit_adapter`」这一对。它验的是**翻译**，不是规划 ——
规划、逆解、避障都是 MoveIt 的活，那个替身不做，所以这里也不验。

本体是 `check()`（不符合预期就 assert），由 test/test_pose.py 调用。单独跑：

    pytest src/robot_sdk_tests/test/test_pose.py -s
"""

import rclpy
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from robot_sdk_interfaces.action import MoveToPose

GOAL_TOPIC = "/fake_move_group/goal"
GROUP = "arm"
EE_LINK = "gripper_link"
POSITION_TOLERANCE = 0.01      # 和适配器的默认参数对齐


class Probe(Node):
    def __init__(self, action_name):
        super().__init__("probe_pose")
        self.client = ActionClient(self, MoveToPose, action_name)
        self.received = []
        # ⚠️ latch（TRANSIENT_LOCAL）：我们是**先发目标、后读**的，
        #    用默认的 VOLATILE 会一条都收不到 —— 这坑在代价地图上踩过。
        self.create_subscription(
            MoveGroup.Goal, GOAL_TOPIC, self.received.append,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def wait(self, future, timeout=30.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"等 {timeout:.0f} 秒没有回应")
        return future.result()

    def drain(self, seconds=1.0):
        """把订阅到的旧目标收干净 —— 每条判据只该看它自己那一次的目标。"""
        self.received.clear()
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def go(self, x, y, z, orientation=(0.0, 0.0, 0.0, 1.0), any_orientation=False,
           max_velocity=0.0):
        goal = MoveToPose.Goal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        (goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
         goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w) = orientation
        goal.any_orientation = any_orientation
        goal.max_velocity = max_velocity

        handle = self.wait(self.client.send_goal_async(goal))
        assert handle.accepted, f"目标 ({x}, {y}, {z}) 该被接受"
        # 结果回来之后再多转一会儿，让 latch 那帧也到。
        result = self.wait(handle.get_result_async()).result
        end = self.get_clock().now().nanoseconds + int(0.5 * 1e9)
        while not self.received and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return result

    def last_goal(self):
        assert self.received, \
            f"假 move_group 没收到目标 —— 适配器没把它发下去 / 或者 {GOAL_TOPIC} 没订上"
        return self.received[-1]


def check(action_name="/sdk/move_to_pose"):
    """翻译是否对得上。不符合预期就 assert。"""
    rclpy.init()
    try:
        probe = Probe(action_name)
        assert probe.client.wait_for_server(timeout_sec=30.0), f"等不到动作服务端 {action_name}"
        probe.drain()

        # ── 一、带姿态的目标（any_orientation=False）──
        result = probe.go(0.3, 0.1, 0.5, any_orientation=False, max_velocity=0.2)
        assert result.success, f"该成功到达（'{result.message}'）"

        request = probe.last_goal().request
        assert request.group_name == GROUP, \
            f"规划组该是 '{GROUP}'，实为 '{request.group_name}'"
        assert len(request.goal_constraints) == 1, \
            f"该有且只有一组约束，实为 {len(request.goal_constraints)}"

        constraints = request.goal_constraints[0]
        assert len(constraints.position_constraints) == 1, "位置约束该有且只有一条"
        position = constraints.position_constraints[0]

        assert position.link_name == EE_LINK, \
            f"位置约束该挂到末端 '{EE_LINK}'，实为 '{position.link_name}'"
        assert position.header.frame_id == "map", \
            f"坐标系该跟着目标走（map），实为 '{position.header.frame_id}'"

        primitive = position.constraint_region.primitives[0]
        assert len(primitive.dimensions) >= 1, "球没给半径 —— 那就是个空区域"
        assert abs(primitive.dimensions[0] - POSITION_TOLERANCE) < 1e-6, \
            f"球半径该等于位置容差 {POSITION_TOLERANCE}，实为 {primitive.dimensions[0]}"

        center = position.constraint_region.primitive_poses[0].position
        assert (abs(center.x - 0.3) < 1e-6 and abs(center.y - 0.1) < 1e-6
                and abs(center.z - 0.5) < 1e-6), \
            f"区域中心该是目标点 (0.3, 0.1, 0.5)，实为 ({center.x}, {center.y}, {center.z})"

        offset = position.target_point_offset
        assert abs(offset.x) + abs(offset.y) + abs(offset.z) < 1e-9, \
            "target_point_offset 该是零（末端原点就到那儿，不是偏移过去的）"

        assert len(constraints.orientation_constraints) == 1, \
            "客户带了姿态，姿态约束就该**填**上（否则等于悄悄忽略他的话）"
        orientation = constraints.orientation_constraints[0]
        assert orientation.link_name == EE_LINK, "姿态约束也该挂到末端"
        assert orientation.absolute_x_axis_tolerance > 0.0, "姿态容差该是正数"

        # ── 速度：该翻到 m/s 的那一格，不是比例那一格 ──
        assert abs(request.max_cartesian_speed - 0.2) < 1e-6, \
            f"max_velocity 该原样进 max_cartesian_speed（m/s），实为 {request.max_cartesian_speed}"
        assert request.cartesian_speed_limited_link == EE_LINK, \
            "给速度就该同时点名是哪个 link 被限速"
        assert abs(request.max_velocity_scaling_factor) < 1e-9, \
            "**不该**动那个比例字段（它乘的是关节上限，和「速度」不是同一个量）"

        print(f"✅ 「位姿 → 约束」对得上：组={GROUP} 末端={EE_LINK} "
              f"半径={primitive.dimensions[0]} 姿态=填了 速度={request.max_cartesian_speed} m/s")

        # ── 二、朝向随你（any_orientation=True）──
        probe.drain()
        result = probe.go(0.4, 0.0, 0.5, any_orientation=True)
        assert result.success, f"any_orientation 那条也该成功（'{result.message}'）"

        constraints = probe.last_goal().request.goal_constraints[0]
        assert len(constraints.orientation_constraints) == 0, \
            "any_orientation=True 时**不该**填姿态约束 —— " \
            "填个单位四元数等于「朝基准方向」，那是个真约束，不是「随便」"
        assert len(constraints.position_constraints) == 1, "位置约束该照旧还在"
        print("✅ any_orientation=True 时只给位置约束")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    check()
