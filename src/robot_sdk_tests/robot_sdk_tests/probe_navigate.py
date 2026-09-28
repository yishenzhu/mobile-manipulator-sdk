#!/usr/bin/env python3
"""验收探针：导航那条路 —— 到得了、走得到、取消得掉。

打的是 `ros2 launch robot_sdk_nav2 navigation.launch.py` 起来的**整栈**：
mock 机器人 + `nav2_adapter` + Nav2（planner / controller / behavior / bt_navigator +
map_server）。也就是说这一条同时压到：适配器的翻译、Nav2 的规划与追踪、map_server、
生命周期管理 —— 全仓最复杂的一条链路，此前**零自动覆盖**。

本体是 `check()`（不符合预期就 assert），由 test/test_navigation.py 调用。单独跑：

    pytest src/robot_sdk_tests/test/test_navigation.py -s
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from robot_sdk_interfaces.action import NavigateToPose

HOME = (0.0, 0.0)         # 每条都先回这儿 —— 上一摊把机器人留在哪，不该影响判据
GOAL = (2.0, 1.0)         # 房间 x∈[-5,5] y∈[-4,4] 里，够远又好走
FAR_GOAL = (-4.0, -3.0)   # 取消那一档用：远，跑一半就被喊停
TOLERANCE = 0.15          # 和 nav2_params.yaml 的 xy_goal_tolerance 对齐


class Probe(Node):
    def __init__(self, action_name):
        super().__init__("probe_navigate")
        self.client = ActionClient(self, NavigateToPose, action_name)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def wait(self, future, timeout=60.0):
        # 导航比关节运动慢得多，所以这里的上限比别的探针宽。
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f"等 {timeout:.0f} 秒没有回应")
        return future.result()

    def pose(self, timeout=30.0):
        """从 TF 上读地图坐标系里的位置 —— 光看 `final_distance_error` 说不清
        "它到底是走过去的，还是本来就在那儿"。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.buffer.can_transform("map", "base_footprint", Time()):
                t = self.buffer.lookup_transform("map", "base_footprint", Time())
                return t.transform.translation.x, t.transform.translation.y
            rclpy.spin_once(self, timeout_sec=0.2)
        raise TimeoutError(f"等 {timeout:.0f} 秒拿不到 map→base_footprint")

    def send(self, target, max_speed=0.5, avoid=False):
        goal = NavigateToPose.Goal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = target[0]
        goal.target_pose.pose.position.y = target[1]
        goal.target_pose.pose.orientation.w = 1.0
        goal.max_speed = max_speed
        goal.avoid_obstacles = avoid
        handle = self.wait(self.client.send_goal_async(goal))
        assert handle.accepted, f"目标 {target} 该被接受"
        return handle

    def go(self, target):
        """发一个目标并等它结束，返回结果。"""
        return self.wait(self.send(target).get_result_async()).result


def check(action_name="/sdk/navigate_to_pose"):
    """到得了 + 走得到 + 取消得掉。不符合预期就 assert。"""
    rclpy.init()
    try:
        probe = Probe(action_name)
        assert probe.client.wait_for_server(timeout_sec=30.0), \
            f"等不到动作服务端 {action_name}"

        # 先回原点：不这么做的话，"走了多远"就取决于上一摊把机器人留在哪
        # （`probe_accel` 就在这上面栽过一次）。
        print("  先回原点...")
        assert probe.go(HOME).success, "回原点失败"

        start = probe.pose()
        print(f"  起点 ({start[0]:.2f}, {start[1]:.2f}) → 目标 {GOAL}")

        result = probe.go(GOAL)
        print(f"  success={result.success}  离目标 {result.final_distance_error:.3f} m  "
              f"'{result.message}'")
        assert result.success, "该成功到达"
        assert result.final_distance_error <= TOLERANCE, \
            f"误差 {result.final_distance_error:.3f} 超过容差 {TOLERANCE}"

        end = probe.pose()
        moved = math.dist(start, end)
        print(f"  终点 ({end[0]:.2f}, {end[1]:.2f})，走了 {moved:.2f} m")
        assert moved > 1.0, f"只挪了 {moved:.2f} m —— 这不像走过去的"

        # ── 取消 ──
        print("  发个远目标，跑一半喊停...")
        handle = probe.send(FAR_GOAL)
        time.sleep(2.0)                       # 让它真的跑起来
        cancel_response = probe.wait(handle.cancel_goal_async())
        assert cancel_response.goals_canceling, "取消该被受理"

        result = probe.wait(handle.get_result_async()).result
        print(f"  取消后 success={result.success}  message='{result.message}'")
        assert not result.success, "取消之后不该报成功"

        print("✅ 到得了 + 走得到 + 取消得掉")
    finally:
        rclpy.shutdown()
