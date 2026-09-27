"""把 mock 机器人 + Nav2 + 适配器一起拉起来 —— 真 Nav2 驱动 SDK 契约上的机器人。

    terminal 1:
        ros2 launch robot_sdk_nav2 navigation.launch.py
    terminal 2:
        ros2 run robot_sdk_examples demo_navigation sdk/navigate_to_pose

为什么不直接用 nav2_bringup/navigation_launch.py：它固定拉起 10 个节点
（含 route/docking/collision_monitor/smoother），每个都要配参数，少一个就卡在
生命周期里。这里只起本部署真正需要的四个 + 生命周期管理。

也不需要 SLAM / 地图 / amcl：mock 直接广播 map -> odom -> base_footprint，
相当于"定位永远完美"。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# 生命周期管理器按这个顺序把它们激活起来。map_server 必须**最前** ——
# 全局代价地图的 static_layer 要等它把地图发出来。
NAV2_NODES = ["map_server", "planner_server", "controller_server", "behavior_server", "bt_navigator"]


def generate_launch_description():
    share = get_package_share_directory("robot_sdk_nav2")
    params = os.path.join(share, "config", "nav2_params.yaml")
    room_map = os.path.join(share, "maps", "room.yaml")

    return LaunchDescription(
        [
            # ── 静态地图：全局代价地图的 static_layer 从它拿 ──
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[params, {"yaml_filename": room_map}],
            ),
            # ── 被控对象：模拟机器人（发 TF / 激光，收 cmd_vel）──
            Node(
                package="robot_sdk_sim",
                executable="mock_robot_node",
                name="mock_robot",
                output="screen",
            ),
            # ── 翻译层：SDK 动作 ──▶ Nav2 动作 ──
            Node(
                package="robot_sdk_bridge",
                executable="nav2_adapter",
                name="nav2_adapter",
                output="screen",
            ),
            # ── Nav2 本体（最小集）──
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[params],
            ),
            # ── 谁先启动、什么时候激活 ──
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{"autostart": True, "node_names": NAV2_NODES}],
            ),
        ]
    )
