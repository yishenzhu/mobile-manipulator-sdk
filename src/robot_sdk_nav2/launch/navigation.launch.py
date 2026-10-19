"""把机器人 + Nav2（+ 可选 SLAM）拉起来。

    # mock（默认）：无仿真依赖，定位"永远完美"
    ros2 launch robot_sdk_nav2 navigation.launch.py

    # Gazebo + SLAM：真 TF 链，map→odom 由 slam_toolbox 发，同时在建图
    ros2 launch robot_sdk_nav2 navigation.launch.py backend:=gazebo slam:=true

为什么不直接用 nav2_bringup/navigation_launch.py：它固定拉起 10 个节点（含 route /
docking / collision_monitor / smoother），每个都要配参数，少一个就卡在生命周期里。
这里只起本部署真正需要的几个 + 生命周期管理。

三种拓扑 —— **同一份节点清单**，只换"谁提供机器人 / 地图 / 定位"：
· mock        ：mock_robot_node 直接广播 map→odom；map_server 发预置地图
· gazebo      ：gz + gz_ros2_control；odom→base 由 odom_tf_broadcaster 发（仍用预置地图）
· gazebo+slam ：再加 slam_toolbox ⇒ 它发 map→odom 并把地图发到 /map
                ⇒ 此时**不能**再起 map_server（会有两个 /map 作者）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 生命周期管理器按这个顺序激活。map_server 必须**最前**（static_layer 要等地图）；
# SLAM 模式下它不在列表里（地图改由 slam_toolbox 提供）。
NAV2_NODES = ["map_server", "planner_server", "controller_server",
              "behavior_server", "bt_navigator"]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="mock",
                              description="mock | gazebo（谁提供机器人与 TF）"),
        DeclareLaunchArgument("slam", default_value="false",
                              description="true ⇒ 起 slam_toolbox（发 map→odom 并建图）"),
        DeclareLaunchArgument("headless", default_value="true",
                              description="Gazebo 是否无界面（演示时可 false）"),
        OpaqueFunction(function=setup_nodes),
    ])


def setup_nodes(context):
    be = LaunchConfiguration("backend").perform(context)
    slam = LaunchConfiguration("slam").perform(context) == "true"
    headless = LaunchConfiguration("headless").perform(context)
    use_sim = be == "gazebo"
    if be not in ("mock", "gazebo"):
        raise RuntimeError(f"backend 只能是 mock 或 gazebo，收到 {be!r}")

    share = get_package_share_directory("robot_sdk_nav2")
    params = os.path.join(share, "config", "nav2_params.yaml")
    slam_params = os.path.join(share, "config", "slam_toolbox.yaml")
    room_map = os.path.join(share, "maps", "room.yaml")
    # use_sim_time 只有一处：按 backend 算出，注入所有节点（别在 yaml 里再写一份）。
    # ⚠️ 必须是 **dict**（参数是 name→value 的映射），直接塞 ParameterValue 会报
    #    "‘bool’ object is not iterable"（实测）。
    sim = {"use_sim_time": ParameterValue(use_sim, value_type=bool)}

    actions = []

    # ── 被控对象 / 世界：二选一。在 OpaqueFunction 里直接 if，**不要**用 IfCondition ——
    #    它只接受 substitution，塞 Python bool 会报 "'bool' object is not iterable"（实测）──
    if be == "gazebo":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("robot_sdk_gazebo"), "launch", "gazebo.launch.py")),
            launch_arguments={"headless": headless, "enable_camera": "false"}.items()))
    else:
        actions.append(Node(package="robot_sdk_sim", executable="mock_robot_node",
                            name="mock_robot", output="screen", parameters=[sim]))

    # ── 地图 + 定位：SLAM 时由 slam_toolbox 提供 map→odom 与 /map；否则用预置地图 ──
    if slam:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
            launch_arguments={"use_sim_time": str(use_sim).lower(),
                              "slam_params_file": slam_params}.items()))
    else:
        actions.append(Node(package="nav2_map_server", executable="map_server",
                            name="map_server", output="screen",
                            parameters=[params, sim, {"yaml_filename": room_map}]))

    # ── SDK 动作 ──▶ Nav2 动作的翻译层（两种后端都用）──
    actions.append(Node(package="robot_sdk_bridge", executable="nav2_adapter",
                        name="nav2_adapter", output="screen", parameters=[sim]))

    # ── Nav2 本体（最小集）──
    for pkg, exe in [("nav2_planner", "planner_server"),
                     ("nav2_controller", "controller_server"),
                     ("nav2_behaviors", "behavior_server"),
                     ("nav2_bt_navigator", "bt_navigator")]:
        actions.append(Node(package=pkg, executable=exe, name=exe, output="screen",
                            parameters=[params, sim]))

    node_names = [n for n in NAV2_NODES if not (slam and n == "map_server")]
    actions.append(Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", output="screen",
        parameters=[{"autostart": True, "node_names": node_names}, sim]))
    return actions
