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
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
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

    # ⚠️ 两处覆盖都必要：`params_file` 必须是**存在的绝对路径**（空/相对会让控制器全加载失败）；
    #    `open_loop_control` 关掉 —— 假硬件那条必须开，但 Gazebo 有真实状态，开环会让控制器
    #    **没等到位就报成功** ⇒ MoveIt 拿没稳定的状态校验下一条轨迹 ⇒ "start point deviates"。
    with open(params_path) as handle:
        config = yaml.safe_load(handle)
    controller_manager_params = config.setdefault("controller_manager", {}).setdefault(
        "ros__parameters", {})
    controller_manager_params["params_file"] = params_path
    config.setdefault("arm_controller", {}).setdefault("ros__parameters", {})[
        "open_loop_control"
    ] = False
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
    def spawner(controller, extra_args=()):
        return ExecuteProcess(
            cmd=["ros2", "run", "controller_manager", "spawner", controller,
                 "--controller-manager", "/controller_manager",
                 "--controller-manager-timeout", "60",
                 "--service-call-timeout", "60",
                 "--switch-timeout", "60", *extra_args],
            output="screen",
        )

    spawn_broadcaster = spawner("joint_state_broadcaster")
    spawn_arm = spawner("arm_controller")
    # 底盘：标准 diff_drive_controller（原来由 gz 的 DiffDrive 插件驱动）。
    # ⚠️ 控制器话题是 ~/cmd_vel / ~/odom（节点名带控制器名）⇒ 必须重映射回 /cmd_vel、/odom，
    #    否则 Nav2 与 odom_tf_broadcaster 都认不出。只给控制器传 remap 要用
    #    spawner 的 --controller-ros-args（controller_manager 自己的 --remap 已废弃）。
    spawn_diff_drive = spawner(
        "diff_drive_controller",
        ["--controller-ros-args", "-r ~/cmd_vel:=/cmd_vel -r ~/odom:=/odom"])

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
        # gz_ros2_control 的插件要等机器人被生成之后才会建 controller_manager，所以推迟一点；
        # 两个 spawner 各自独立起、各自等（--controller-manager-timeout），**不串成链** ——
        # 串的话前一个失败也会照样起后一个，留下"看着活着、实则半配置"的栈（实测踩过）。
        TimerAction(period=12.0, actions=[spawn_broadcaster, spawn_arm, spawn_diff_drive]),
    ])
