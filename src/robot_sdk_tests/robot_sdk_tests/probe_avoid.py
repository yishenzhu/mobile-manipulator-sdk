#!/usr/bin/env python3
"""验收探针：avoid_obstacles —— 把目标放到墙外，看它撞不撞。

房间是 x ∈ [-5, 5]、y ∈ [-4, 4]。目标取 (6.0, 0.0) —— **在墙外**。

    avoid_obstacles = True  → 朝目标看一眼，发现墙挡着，**一步都不走**，报失败
    avoid_obstacles = False → 直冲，一路走到 x=6.0（穿过墙）

本体是 `check()`（不符合预期就 assert），由 test/test_probes.py 调用。单独跑：

    pytest src/robot_sdk_tests/test/test_probes.py -s -k avoid
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_sdk_interfaces.action import NavigateToPose

TARGET = (6.0, 0.0)


class Probe(Node):
    def __init__(self, action_name):
        super().__init__("probe_avoid")
        self.client = ActionClient(self, NavigateToPose, action_name)

    def wait(self, future, timeout=30.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"等 {timeout:.0f} 秒没有回应")
        return future.result()

    def go(self, avoid):
        goal = NavigateToPose.Goal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = TARGET[0]
        goal.target_pose.pose.position.y = TARGET[1]
        goal.target_pose.pose.orientation.w = 1.0
        goal.max_speed = 0.5
        goal.avoid_obstacles = avoid

        handle = self.wait(self.client.send_goal_async(goal))
        assert handle.accepted, "目标该被接受"
        return self.wait(handle.get_result_async()).result

    def run(self, avoid):
        result = self.go(avoid)
        print(f"  avoid_obstacles={str(avoid):<5} → success={int(result.success)}  "
              f"离目标还剩 {result.final_distance_error:.2f} m  "
              f"'{result.message}'")
        return result


def check(action_name="/navigate_to_pose"):
    """两种取值各验一条：true 不走，false 走到。不符合预期就 assert。"""
    rclpy.init()
    try:
        print(f"目标 {TARGET}（房间只到 x=5.0，这个目标在墙外）")
        node = Probe(action_name)
        assert node.client.wait_for_server(timeout_sec=10.0), \
            f"等不到动作服务端 {action_name}"

        blocked = node.run(True)
        assert not blocked.success, "avoid_obstacles=true 走不到就该报失败（不然就是撞过去了）"

        straight = node.run(False)
        assert straight.success, "avoid_obstacles=false 该一路走到（这个模拟器不拦）"
        print("✅ avoid_obstacles 的两种取值：都对")
    finally:
        rclpy.shutdown()
