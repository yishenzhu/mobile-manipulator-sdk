"""把运动链路的两头拉起来：假的下游 + 适配器。探针要打的就是它俩。

    ros2 launch robot_sdk_tests motion_chain.launch.py
    # 另一个终端：
    ros2 run robot_sdk_tests probe_motion
    ros2 run robot_sdk_tests probe_accel /sdk/move_to_joint_positions

导航那条链路不在这里 —— 用 `robot_sdk_nav2` 的 `navigation.launch.py`（整栈），
或者只起 `ros2 run robot_sdk_sim mock_robot_node`（打 mock 自己）。

`probe_wrap` 需要连续关节的声明，那是个数组参数，从命令行传更清楚：

    ros2 run robot_sdk_bridge motion_adapter --ros-args -p 'continuous_joints:=[shoulder_pan_joint]'
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # ── 假的下游：说 FollowJointTrajectory 的关节轨迹控制器替身 ──
            Node(
                package="robot_sdk_sim",
                executable="joint_trajectory_mock",
                name="joint_trajectory_mock",
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
