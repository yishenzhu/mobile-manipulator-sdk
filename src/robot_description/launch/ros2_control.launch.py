"""真 ros2_control：起 controller_manager，再拉起两个控制器。

    ros2 launch robot_description ros2_control.launch.py

它替代了之前那个手写的替身（`robot_sdk_sim` 的 `joint_trajectory_mock`）：
  · 硬件   = `mock_components/GenericSystem`（URDF 的 <ros2_control> 里声明）—— 假硬件，不需要 Gazebo
  · 控制器 = `joint_trajectory_controller/JointTrajectoryController` —— **真货**
  · 发状态 = `joint_state_broadcaster` —— `/joint_states` 由它发（替身不再需要）

链路完整形态：
    ros2 launch robot_description ros2_control.launch.py      ← 执行侧
    ros2 launch robot_description move_group.launch.py        ← 规划侧（会找 arm_controller）
    ros2 run robot_sdk_bridge moveit_adapter ...              ← 翻译层

⚠️ 顺序上**先起这个、再起 move_group 更稳** —— move_group 启动时就要连控制器；
   反过来的话它会给一条 "Unable to connect to controller" 之类的抱怨，
   等控制器起来之后还要手动触发一次重连（`/controller_manager` 重启不是必需的，但省事）。
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

    # ⚠️⚠️ 为什么这条不起眼的 rsp 是**必须的**：
    #    这个版本的 controller_manager 是去 `/robot_description` **话题**上拿机器人描述的
    #    （日志原话："Subscribing to '/robot_description' topic for robot description"），
    #    **不是**从参数拿。只把 `robot_description` 当参数传给它，它会一直打印
    #    "Waiting for data on 'robot_description' topic to finish initialization"，
    #    永远不加载硬件、控制器也永远起不来（`list_controllers` 服务根本不存在）。
    #    robot_state_publisher 发的就是那个话题（而且是 TRANSIENT_LOCAL，后起的也收得到）。
    #
    # ⚠️ 它顺带也发关节 TF —— 这一点和 move_group.launch.py 里那个 rsp 是**重复的**。
    #    两个一起跑会看到 TF_REPEATED_DATA 的警告（值完全一样，不影响结果，但不好看）。
    #    要干净的话：让这条 launch 负责 rsp，把 move_group.launch.py 里那个去掉。
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # controller_manager 就是 ros2_control_node 这个可执行文件，它靠 --params-file 拿到
    # config/ros2_control.yaml。注意下面用的是一份**运行时生成的临时副本**，原因见那段注释。
    # ⚠️⚠️ 生成一份配置副本，并把 `params_file` 补成**本文件的绝对路径**。
    #
    #   为什么非这样不可（全部是实测结论，不是猜的）：
    #     controller_manager 加载控制器时，要把"参数文件"转发给每个控制器节点，
    #     拼成 "--ros-args --params-file <那个值>"。而那个值取自本文件里
    #     controller_manager.ros__parameters.params_file。三种给法全试过：
    #
    #       · 这份文件里**没有这个键**              ⇒ 转发 = 空 ⇒ 失败
    #             Couldn't parse trailing --params-file flag. No file path provided.
    #       · 有键，但值是**相对路径**（如 "ros2_control.yaml"）
    #                                              ⇒ 转发 = 空 ⇒ 失败（同上）
    #       · 有键，且值是**存在的绝对路径**        ⇒ 转发 = 该路径 ⇒ 控制器 Loaded ✅
    #
    #     ⇒ 值必须是"存在的绝对路径"，可配置文件里根本没法知道自己被装到哪儿了，
    #       所以在这里读出来补上、写成一份临时文件，再用它启动。
    #       （临时文件里含全部控制器的 .ros__parameters，正好也是控制器节点要读的东西。）
    #
    #   也试过用 `ros2 run ... --ros-args -p params_file:=<绝对路径>` 代替：
    #   **无效** —— 参数是设上了，但转发依旧为空，它读的是文件里的键。
    #
    #   两种失败的症状都极具误导性：硬件明明 "Successful 'activate' of hardware"，
    #   而 `ros2 control list_controllers` 只说 "No controllers are currently loaded!"，
    #   很容易误判成硬件/插件/URDF 的问题。
    with open(params_path) as handle:
        config = yaml.safe_load(handle)
    config.setdefault("controller_manager", {}).setdefault("ros__parameters", {})[
        "params_file"
    ] = params_path

    fd, generated_params = tempfile.mkstemp(prefix="ros2_control_", suffix=".yaml")
    with os.fdopen(fd, "w") as handle:
        yaml.safe_dump(config, handle)

    # 直接用二进制全路径启动，不经 `ros2 run` 包装（那会把 --ros-args 那段的形态改掉）。
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

    # 用 spawner 起控制器，而不是写死在 yaml 里 —— 这是 ros2_control 的常规做法：
    # 控制器可以随时加载/卸载/切换，spawner 走的是 controller_manager 的服务。
    # ⚠️⚠️ 这里**必须**用 ExecuteProcess，不能用 Node：
    #    spawner 本身不接任何 ROS 参数，而 launch_ros 的 `Node` 无论怎样都会往命令行
    #    尾部补一个光秃秃的 `--ros-args` ⇒ spawner 的参数解析器把它当成
    #    "--params-file 后面缺个路径"，直接抛 RCLInvalidROSArgsError 退出：
    #      Couldn't parse trailing --params-file flag. No file path provided.
    #    结果是两个 spawner 全 exit 1、一个控制器都没加载
    #    （`ros2 control list_controllers` 只会说 "No controllers are currently loaded!"）。
    #    ExecuteProcess 不掺和 launch_ros 那套参数注入，命令行给什么就是什么。
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
        # 不这么做的话 arm_controller 可能赶在 controller_manager 就绪之前被 spawn。
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_broadcaster, on_exit=[spawn_arm])
        ),
    ])
