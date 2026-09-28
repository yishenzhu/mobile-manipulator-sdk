"""把笛卡尔链路的两头拉起来：假的 move_group + 适配器。探针要打的就是它俩。

    ros2 launch robot_sdk_tests pose_chain.launch.py

自动那边由 test/test_pose.py 的 fixture 负责起停 —— 这里就是它起的东西。

`planning_group` / `end_effector_link` 必须给：它们是**部署相关**的参数，
适配器拿不到就会拒绝目标（那是它的设计，不是 bug）。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # ── 假的下游：说 MoveGroup 的 move_group 替身（不做规划）──
            Node(
                package="robot_sdk_sim",
                executable="move_group_mock",
                name="move_group_mock",
                output="screen",
            ),
            # ── 翻译层：SDK 的"位姿" ──▶ MoveGroup 的"约束" ──
            Node(
                package="robot_sdk_bridge",
                executable="moveit_adapter",
                name="moveit_adapter",
                output="screen",
                parameters=[{
                    "planning_group": "arm",
                    "end_effector_link": "gripper_link",
                }],
            ),
        ]
    )
