"""真 ros2_control：起 controller_manager + 两个控制器。

    ros2 launch robot_description ros2_control.launch.py

先起这个、再起 move_group.launch.py（它启动时就要连控制器）。
硬件用 mock_components/GenericSystem，不需要 Gazebo。
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robot_description")
    urdf_path = os.path.join(share, "urdf", "mobile_manipulator.urdf")
    params_path = os.path.join(share, "config", "ros2_control.yaml")

    with open(urdf_path) as handle:
        robot_description = handle.read()

    # ⚠️ controller_manager 从 `/robot_description` **话题**拿描述（不是参数），
    #    所以这条 rsp 必须有 —— 少了它硬件永远不加载、控制器也起不来。
    #    它顺带发的 TF 和 move_group.launch.py 里那个重复，只有 TF_REPEATED_DATA 警告。
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # ⚠️ 生成配置副本，把 `params_file` 补成本文件的**绝对路径**（controller_manager 拿它
    #    转发给各控制器节点；空值或相对路径都会让控制器全部加载失败，而硬件却显示 activate 成功）。
    #    实测：用 `-p params_file:=<路径>` 代替不了它 —— 它读的是文件里的键。
    with open(params_path) as handle:
        config = yaml.safe_load(handle)
    config.setdefault("controller_manager", {}).setdefault("ros__parameters", {})[
        "params_file"
    ] = params_path

    fd, generated_params = tempfile.mkstemp(prefix="ros2_control_", suffix=".yaml")
    with os.fdopen(fd, "w") as handle:
        yaml.safe_dump(config, handle)

    # 用二进制全路径启动，不经 `ros2 run` 包装。
    controller_manager = ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix("controller_manager"),
                "lib", "controller_manager", "ros2_control_node",
            ),
            "--ros-args", "--params-file", generated_params,
        ],
        output="screen",
    )

    # ⚠️ 必须用 ExecuteProcess：launch_ros 的 `Node` 会往命令行尾部补一个光秃秃的 `--ros-args`，
    #    spawner 把它当成 "--params-file 缺路径" 直接退出 ⇒ 一个控制器都加载不了。
    spawn_broadcaster = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster",
             "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_arm = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "arm_controller",
             "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        spawn_broadcaster,
        # spawner 起完就退出 ⇒ 拿它的退出当"可以起下一个了"的信号。
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_broadcaster, on_exit=[spawn_arm])
        ),
    ])
