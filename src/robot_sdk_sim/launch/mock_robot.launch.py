from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
                "robot_description_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_description"), "urdf", "mobile_manipulator.urdf"]
                ),
                description=(
                    "机器人描述：限位/折角由它决定，RViz 的 3D 模型也订它。"
                    "不给 ⇒ 不判限位、也看不到机器人本体。"
                ),
            ),
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
                        # ⚠️ 必须传：不传就没有限位、也没有 /robot_description。
                        # tests 的 fixture 一直是手动传的，走 launch 反而漏了（两处行为不一致）。
                        "robot_description_file": LaunchConfiguration(
                            "robot_description_file"
                        ),
                    }
                ],
            ),
            # ★ 机器人本体各 link 的 TF 由它发（读 URDF + /joint_states）。
            #   没有它，RViz 只会看到 map/odom/base_footprint 与传感器那几个静态帧，
            #   手臂每根连杆都报 "No transform from [x] to [map]"（实测）。
            #   ⚠️ 它只发 URDF 里的 link 帧，不动 mock 自己广播的世界帧。
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(
                                [
                                    "cat ",
                                    PathJoinSubstitution(
                                        [
                                            FindPackageShare("robot_description"),
                                            "urdf",
                                            "mobile_manipulator.urdf",
                                        ]
                                    ),
                                ]
                            ),
                            value_type=str,
                        ),
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
