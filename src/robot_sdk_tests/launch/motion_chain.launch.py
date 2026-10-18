"""把运动链路的两头拉起来：假的下游 + 适配器。探针要打的就是它俩。

    ros2 launch robot_sdk_tests motion_chain.launch.py

自动那边由 test/test_adapter.py 的 fixture 负责起停 —— 这里就是它起的东西。

导航那条链路不在这里 —— 用 `robot_sdk_nav2` 的 `navigation.launch.py`（整栈），
或者只起 `ros2 run robot_sdk_sim mock_robot_node`（打 mock 自己）。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # ── 假的下游：说 FollowJointTrajectory 的关节轨迹控制器替身 ──
            # ⚠️ 这条链里没有 mock_robot_node ⇒ 由它当 /joint_states 的**唯一**发布者。
            #    （与 mock 节点同起时保持默认 false，否则会出现两个发布者 ✗）
            Node(
                package="robot_sdk_sim",
                executable="joint_trajectory_mock",
                name="joint_trajectory_mock",
                parameters=[{"publish_joint_states": True}],
                output="screen",
            ),
            # ── 翻译层：SDK 动作 ──▶ FollowJointTrajectory ──
            Node(
                package="robot_sdk_bridge",
                executable="motion_adapter",
                name="motion_adapter",
                output="screen",
            ),
        ]
    )
