"""真 move_group：把 URDF + SRDF + kinematics + 控制器喂给它。

    ros2 launch robot_description move_group.launch.py

⚠️ 这一条不含控制器本体 —— 要真执行得**先起执行侧**，再起这个：
       ros2 launch robot_description ros2_control.launch.py
⚠️ `config/joint_limits.yaml` 和 `pilz_cartesian_limits.yaml` 必须存在，缺了 launch 直接失败。
"""

from launch import LaunchDescription
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
        generate_rsp_launch(moveit_config),
        generate_static_virtual_joint_tfs_launch(moveit_config),
        generate_move_group_launch(moveit_config),
    ])
