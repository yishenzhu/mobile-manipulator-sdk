from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 注意：这里把命名空间交给**节点的 namespace**，而不是一个名为
    # `robot_namespace` 的参数。
    #
    # 原因：SDK 不再自己拼话题名前缀（`RobotConfig::resolve()` 已删除），
    # 而是依赖 rcl 按节点命名空间展开相对名。所以"换命名空间"的正确做法
    # 就是让节点建在那个命名空间里 —— 参数传进去没用。
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
