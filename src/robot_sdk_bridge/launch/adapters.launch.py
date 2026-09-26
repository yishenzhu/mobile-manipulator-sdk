from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sdk_action_name",
                default_value="sdk/navigate_to_pose",
                description="SDK 侧的导航动作名，必须与 Nav2 自己的名字不同。",
            ),
            DeclareLaunchArgument(
                "nav2_action_name",
                default_value="navigate_to_pose",
                description="Nav2 提供的导航动作名。",
            ),
            DeclareLaunchArgument(
                "default_frame",
                default_value="map",
                description="目标没写 frame_id 时按这个帧处理。",
            ),
            Node(
                package="robot_sdk_bridge",
                executable="nav2_adapter",
                name="nav2_adapter",
                output="screen",
                parameters=[
                    {
                        "sdk_action_name": LaunchConfiguration("sdk_action_name"),
                        "nav2_action_name": LaunchConfiguration("nav2_action_name"),
                        "default_frame": LaunchConfiguration("default_frame"),
                    }
                ],
            ),
        ]
    )
