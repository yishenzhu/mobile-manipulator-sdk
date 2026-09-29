from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 命名空间交给**节点的 namespace**，不是 `node_namespace` 参数：
    # SDK 不再自己拼话题名前缀（`resolve()` 已删），相对名由 rcl 按节点命名空间展开，
    # 所以必须让节点建在那个命名空间里 —— 参数传进去没用。
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_sdk_sim",
                executable="mock_robot_node",
                name="mock_robot",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
        ]
    )
