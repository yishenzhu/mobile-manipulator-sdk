"""真 move_group：把 URDF + SRDF + kinematics + 控制器喂给它，其余（ompl / joint_limits）用默认。

    ros2 launch robot_description move_group.launch.py

为什么只写这么几行：`MoveItConfigsBuilder` 的每个方法名就是"它要你给什么"，
而**没调的那些它有自己的默认** ——
  · `planning_pipelines`  不给 ⇒ 自动用 ompl
⚠️ 但有两处**不调就会出问题**：
  · `joint_limits` —— 不调它，builder 照样会去找 `config/joint_limits.yaml`，
    找不到就让 launch 失败（实测 `ParameterBuilderFileNotFoundError`）。
    ⇒ 那个文件必须有，**内容不能是空的**（空 = 没有速度/加速度上限 ⇒
    `AddTimeOptimalParameterization` 报 FAILURE ⇒ MoveGroup 回 99999）。
  · `moveit_controllers` —— 不调它 ⇒ 「No controller_names specified」⇒
    **只能规划、不能执行**（MoveGroup 回 -4 CONTROL_FAILED）。

起三个东西：
  · robot_state_publisher          —— 发关节的 TF（move_group 要它）
  · static virtual joint TF        —— world → base_footprint（SRDF 里那条 virtual_joint）
  · move_group                     —— 规划本体

⚠️ 这一条**不含控制器本体**（move_group 只知道"该找谁"，不知道"它在不在"）。
   ⇒ 要它真执行，得另起一个 `FollowJointTrajectory` 服务端；
     本仓库里那是 `robot_sdk_sim` 的 `joint_trajectory_mock`：

       ros2 run robot_sdk_sim joint_trajectory_mock --ros-args \
         -p action_name:=/arm_controller/follow_joint_trajectory \
         -p joints:="[shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_joint]"

   ⚠️ 起它时**不要**再起 `mock_robot_node` —— 两个都在发 `/joint_states`，会打架。
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
        # ⚠️ 控制器配置走 `trajectory_execution` —— 这是**官方那个入口**，
        #    默认文件名就是 `config/moveit_controllers.yaml`。
        #    踩过的两个坑：① `MoveItConfigsBuilder` **没有** `moveit_controllers()` 方法；
        #    ② `MoveItConfigs` 是 dataclass，**没有** `moveit_controllers` 字段，赋值会 AttributeError；
        #    ③ `moveit_configs_utils.load_yaml` 这个版本也不存在。
        #    不配的后果：move_group warn「No controller_names specified」⇒
        #    **只能规划、不能执行**（MoveGroup 回 -4 CONTROL_FAILED）。
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription([
        generate_rsp_launch(moveit_config),
        generate_static_virtual_joint_tfs_launch(moveit_config),
        generate_move_group_launch(moveit_config),
    ])
