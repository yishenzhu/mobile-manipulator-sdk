"""真 move_group：把 URDF + SRDF + kinematics + 控制器喂给它。

    ros2 launch robot_description move_group.launch.py
    ros2 launch robot_description move_group.launch.py use_sim_time:=true   # 配 Gazebo 时

⚠️ 这一条不含控制器本体 —— 要真执行得**先起执行侧**，再起这个：
       ros2 launch robot_description ros2_control.launch.py
⚠️ `config/joint_limits.yaml` 和 `pilz_cartesian_limits.yaml` 必须存在，缺了 launch 直接失败。
⚠️ 配 Gazebo（robot_sdk_gazebo）时必须 `use_sim_time:=true` —— 仿真时钟从 0 起、与墙钟
   差着 1.7e9 秒，混用会让 TF 查询和轨迹时间全错。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    moveit_config = (
        # file_path 是**相对 robot_description 的 share 目录**的。
        MoveItConfigsBuilder("mobile_manipulator", package_name="robot_description")
        .robot_description(file_path="urdf/mobile_manipulator.urdf")
        .robot_description_semantic(file_path="srdf/mobile_manipulator.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # ⚠️ 控制器配置走 `trajectory_execution`（官方入口，默认文件名就是它）。
        #    踩过的坑：builder 没有 `moveit_controllers()` 方法，`MoveItConfigs` 也没这个字段，
        #    这个版本的 `moveit_configs_utils.load_yaml` 不存在。
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="配 Gazebo 时设 true。"),
        # MoveIt 执行前会拿轨迹起点和"当前状态"比对，差超过这个值就拒执行（报 CONTROL_FAILED）。
        # 默认 0.01 是给真机的；仿真的跟踪误差本来就大，配 Gazebo 时放宽到 ~0.1。
        DeclareLaunchArgument("allowed_start_tolerance", default_value="0.01",
                              description="轨迹起点与当前状态的最大允许偏差（弧度/米）。"),
        # 同作用域里的节点都会拿到它们 —— rsp / move_group / 静态 TF 广播器都需要。
        SetParameter(name="use_sim_time",
                     value=ParameterValue(LaunchConfiguration("use_sim_time"),
                                          value_type=bool)),
        SetParameter(name="trajectory_execution.allowed_start_tolerance",
                     value=ParameterValue(LaunchConfiguration("allowed_start_tolerance"),
                                          value_type=float)),
        generate_rsp_launch(moveit_config),
        generate_static_virtual_joint_tfs_launch(moveit_config),
        generate_move_group_launch(moveit_config),
    ])
