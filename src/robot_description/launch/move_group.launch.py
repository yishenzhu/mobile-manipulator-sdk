"""真 move_group：把 URDF + SRDF + kinematics 喂给它，其余（ompl / joint_limits）用默认。

    ros2 launch robot_description move_group.launch.py

为什么只写这么几行：`MoveItConfigsBuilder` 的每个方法名就是"它要你给什么"，
而**没调的那些它有自己的默认** ——
  · `planning_pipelines`  不给 ⇒ 自动用 ompl
  · `trajectory_execution` 不给 ⇒ 只是 warn（找不到 config/*_controllers.yaml）
⚠️ 但 `joint_limits` **不一样** —— 不调它，builder 照样会去找 `config/joint_limits.yaml`，
   找不到就让 launch 失败（实测 `ParameterBuilderFileNotFoundError`）。
   ⇒ 所以那个文件必须有，**内容可以是空的**（空 = 不覆盖，全按 URDF 的 <limit>）。

起三个东西：
  · robot_state_publisher          —— 发关节的 TF（move_group 要它）
  · static virtual joint TF        —— world → base_footprint（SRDF 里那条 virtual_joint）
  · move_group                     —— 规划本体

⚠️ 这一条**不含控制器** ⇒ 只能规划，不能真的动。
   要它真动起来得再加 ros2_control / 一个 fake controller —— 那是另一件事。
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
        # 包名给 robot_description —— URDF / SRDF / kinematics 都住在它的 share 目录里，
        # 下面那几个 file_path 是**相对那个目录**的。
        MoveItConfigsBuilder("mobile_manipulator", package_name="robot_description")
        .robot_description(file_path="urdf/mobile_manipulator.urdf")
        .robot_description_semantic(file_path="srdf/mobile_manipulator.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription([
        generate_rsp_launch(moveit_config),
        generate_static_virtual_joint_tfs_launch(moveit_config),
        generate_move_group_launch(moveit_config),
    ])
