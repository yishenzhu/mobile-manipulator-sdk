#!/usr/bin/env python3
"""验收探针：avoid_obstacles —— 把目标放到墙外，看它撞不撞。

房间是 x ∈ [-5, 5]、y ∈ [-4, 4]。目标取 (6.0, 0.0) —— **在墙外**。

    avoid_obstacles = True  → 朝目标看一眼，发现墙挡着，**一步都不走**，报失败
    avoid_obstacles = False → 直冲，一路走到 x=6.0（穿过墙）

先跑 True 再跑 False：前者不动，后者才是"撞过去"。判据就是这两条。

    ros2 run robot_sdk_tests probe_avoid
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_sdk_interfaces.action import NavigateToPose

TARGET = (6.0, 0.0)


class Probe(Node):
    def __init__(self, action_name):
        super().__init__("probe_avoid")
        self.client = ActionClient(self, NavigateToPose, action_name)

    def wait(self, future):
        rclpy.spin_until_future_complete(self, future)
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
        if not handle.accepted:
            return None
        return self.wait(handle.get_result_async()).result

    def run(self, avoid):
        result = self.go(avoid)
        if result is None:
            print(f"  avoid_obstacles={str(avoid):<5} → 目标被拒")
            return None
        print(f"  avoid_obstacles={str(avoid):<5} → success={int(result.success)}  "
              f"离目标还剩 {result.final_distance_error:.2f} m  "
              f"'{result.message}'")
        return result


def main():
    rclpy.init()
    action_name = sys.argv[1] if len(sys.argv) > 1 else "/navigate_to_pose"
    node = Probe(action_name)
    if not node.client.wait_for_server(timeout_sec=10.0):
        print(f"❌ 等不到动作服务端 {action_name}")
        return 1

    print(f"目标 {TARGET}（房间只到 x=5.0，这个目标在墙外）")
    blocked = node.run(True)
    straight = node.run(False)

    node.destroy_node()
    rclpy.shutdown()

    # 判据：true 必须"不走"（失败），false 必须"走到了"（成功）。
    ok = (blocked is not None and not blocked.success
          and straight is not None and straight.success)
    print("✅ 验收通过" if ok else "❌ 验收未通过")
    return 0 if ok else 1


def cli():
    """console_scripts 的入口 —— 它**不看** main() 的返回值，得自己转成退出码。"""
    sys.exit(main())


if __name__ == "__main__":
    sys.exit(main())
