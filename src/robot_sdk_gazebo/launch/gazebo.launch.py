"""在 Gazebo 里跑参考机器人：起仿真 + 桥接 + odom TF，并让 ros2_control 驱动机器人。

    ros2 launch robot_sdk_gazebo gazebo.launch.py
    ros2 launch robot_sdk_gazebo gazebo.launch.py headless:=true

⚠️ URDF 用 `hardware:=gazebo` 展开（硬件插件换成 gz_ros2_control）⇒ 手臂由 ros2_control
   驱动，MoveIt 那条链路在这里也走得通。控制器由 gz 插件建的 controller_manager 托管，
   所以**不另起 ros2_control_node**，但要另外用 spawner 加载/激活它们。
⚠️ 渲染式的激光/深度相机要 EGL 上下文（用户得在 video / render 组里）。
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")

    share = get_package_share_directory("robot_description")
    params_path = os.path.join(share, "config", "ros2_control.yaml")

    # ⚠️ 和 ros2_control.launch.py 同一个道理：controller_manager 转发给控制器节点的那个
    #    params_file 必须是**存在的绝对路径**，空值或相对路径会让控制器全部加载失败。
    with open(params_path) as handle:
        config = yaml.safe_load(handle)
    config.setdefault("controller_manager", {}).setdefault("ros__parameters", {})[
        "params_file"
    ] = params_path
    fd, generated_params = tempfile.mkstemp(prefix="gz_ros2_control_", suffix=".yaml")
    with os.fdopen(fd, "w") as handle:
        yaml.safe_dump(config, handle)

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "urdf", "mobile_manipulator.urdf.xacro"])
    world_file = PathJoinSubstitution(
        [FindPackageShare("robot_sdk_gazebo"), "worlds", "sdk_demo.sdf"])
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("robot_sdk_gazebo"), "config", "ros_gz_bridge.yaml"])

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file,
                 " hardware:=gazebo",
                 " ros2_control_params:=", generated_params,
                 " enable_camera:=", LaunchConfiguration("enable_camera")]),
        value_type=str)

    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": ["-r -s --headless-rendering ", world_file]}.items(),
        condition=IfCondition(headless))
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": ["-r ", world_file]}.items(),
        condition=UnlessCondition(headless))

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "mobile_manipulator",
                   "-allow_renaming", "false"],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
    )

    odom_tf = Node(
        package="robot_sdk_gazebo",
        executable="odom_tf_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ⚠️ gz_ros2_control 的插件只建 controller_manager，**不自动加载控制器** ⇒ 还得用 spawner 起。
    #    ⚠️ 必须用 ExecuteProcess：launch_ros 的 Node 会往命令行尾部补一个光秃秃的 `--ros-args`，
    #    spawner 会把它当成"--params-file 缺路径"直接退出。
    spawn_broadcaster = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster",
             "--controller-manager", "/controller_manager",
             "--controller-manager-timeout", "60"],
        output="screen",
    )
    spawn_arm = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "arm_controller",
             "--controller-manager", "/controller_manager",
             "--controller-manager-timeout", "60"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="不开 GUI 只跑物理（无显示器时需要）。"),
        DeclareLaunchArgument(
            "enable_camera",
            default_value=PythonExpression(["'false' if '", headless, "' == 'true' else 'true'"]),
            description=("是否生成深度相机；headless 下默认关掉省带宽。激光和相机都是"
                         "渲染式的，都需要可用的 EGL/GPU 上下文。"),
        ),
        gz_headless,
        gz_gui,
        robot_state_publisher,
        bridge,
        odom_tf,
        # 给 Gazebo 一点时间再请求生成机器人。
        TimerAction(period=4.0, actions=[spawn_robot]),
        # 再等一会：要等机器人被生成、gz_ros2_control 才把 controller_manager 建起来。
        TimerAction(period=12.0, actions=[spawn_broadcaster]),
        # spawner 起完就退出 ⇒ 拿它的退出当"可以起下一个了"的信号。
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_broadcaster, on_exit=[spawn_arm])
        ),
    ])
