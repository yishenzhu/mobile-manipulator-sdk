from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 命名空间交给**节点的 namespace**，不是 `node_namespace` 参数：
    # SDK 不再自己拼话题名前缀（`resolve()` 已删），相对名由 rcl 按节点命名空间展开，
    # 所以必须让节点建在那个命名空间里 —— 参数传进去没用。
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="顺便起 rviz2（Fixed Frame=map，配置在 rviz/mock.rviz）。需要图形环境。",
            ),
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
            # rviz2 **不进命名空间**：它要看整张图。跟进去反而订不到 /tf，
            # 画面上会什么都没有（TF 订阅用的是绝对名）。
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [FindPackageShare("robot_sdk_sim"), "rviz", "mock.rviz"]),
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
