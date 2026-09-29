# robot_sdk_gazebo

在 Gazebo 里跑参考机器人：起仿真 + 把 gz 话题桥到 ROS + 广播 `odom → base` 的 TF。
**手臂由 ros2_control 驱动**（`gz_ros2_control` 的硬件插件），所以 MoveIt 那条链路在这里也走得通。

## 怎么跑

```bash
ros2 launch robot_sdk_gazebo gazebo.launch.py                 # 带 GUI
ros2 launch robot_sdk_gazebo gazebo.launch.py headless:=true  # 无显示器 / 省内存

# 规划侧（**必须** use_sim_time，否则仿真时钟和墙钟差 1.7e9 秒）
ros2 launch robot_description move_group.launch.py use_sim_time:=true allowed_start_tolerance:=0.1

# 翻译层
ros2 run robot_sdk_bridge moveit_adapter --ros-args \
  -p sdk_action_name:=sdk/move_to_pose -p move_group_action_name:=/move_action \
  -p planning_group:=arm -p end_effector_link:=gripper_base_link -p use_sim_time:=true
```

## 三条链路各自归谁

| 部件 | 谁驱动 |
|---|---|
| 手臂 7 个关节 | ros2_control（`gz_ros2_control/GazeboSimSystem`）⇒ MoveIt 能直接驱 |
| 底盘（差速） | gz 的 `DiffDrive`（**没**进 ros2_control）⇒ 走 `/cmd_vel`、`/odom` 两条桥 |
| 激光 / IMU / 深度相机 | gz 的 Sensors 系统 ⇒ 过桥 |

URDF 用 `hardware:=gazebo` 展开（默认 `mock` 是给 `colcon test` 的假硬件那条）。

## 实测到什么程度

```
✅ 控制器到 active（arm_controller / joint_state_broadcaster）
✅ gz_ros2_control **真的**驱动了手臂：直接发一条轨迹，shoulder_pan 实走 0.4000 rad
   （期望 0.4），其它关节偏移 0.0000，error_code = 0
✅ MoveIt 端到端：MoveToPose 连发 8 条（2 seed × 两种朝向）**全部 success=True**，
   位置误差 6~12 毫米，CONTROL_FAILED = 0
```

## ⚠️ 已知问题（别再踩）

**① 它**不进** `colcon test` —— 这是有依据的决定**：headless 全套要 ~1.1 GB 可用内存、
约 3 分钟，而且**精度依赖物理**（姿态误差实测 0.09~0.21 rad，比假硬件那条的 0.03~0.05 差得多，
因为 gz 的位置控制器在重力下有跟踪误差）。拿它当 CI 判据会既慢又抖。

**② bring-up 曾经只有约一半成功率 —— 已修（2026-09-29）。**
症状：`spawner` 没成功，`list_controllers` 显示控制器 `unconfigured` / `inactive`
（**加载上了但没激活**），栈"看着活着、实则半配置"。
根因是 spawner 的**服务调用与切换超时**在这台机器的负载下不够长。修法两步：
① 两个 spawner **各自独立起**，不再把 arm 挂在 broadcaster 的退出事件上（串成链时前一个失败
也会照样起后一个，级联更糟）；② 两个都加 `--service-call-timeout 60 --switch-timeout 60`。
**修后连续 2 次 bring-up 都干净起来**（修前 4 次里 2 次）。

**③ 车轮的关节状态 —— 已解决。** URDF 里 gz 的 `JointStatePublisher` 只发**车轮**（手臂归
`joint_state_broadcaster`），所以 `/joint_states` 必须**桥**过来，MoveIt 的当前状态才完整。
⚠️ 这里踩过一次并**归因错了**：最初桥上之后整栈全 `-4`，我误判成"桥导致时间戳回跳"就退掉了。
用日志核对才发现真正的问题是 spawner 没激活控制器（即上面的问题②），而 `jump back in time`
在**不带桥**的那轮里反而更多（3663 次 vs 1217 次）。
②修好之后把桥加回来重测：**状态完整性警告 191 → 0、jump back in time → 0、端到端 4/4 成功**。

**④ 内存是硬约束。** 这台机器 5.9 GB 总、起栈后只剩 ~1 GB。同时跑 Gazebo + move_group +
适配器已经接近上限，再叠东西会被 OOM 杀掉（表现为栈莫名其妙整体消失）。

## 和 world 的对应关系

`worlds/sdk_demo.sdf` 的房间是 **x ±5、y ±4**，和 `mock_robot_node.cpp` 的 `kRoomHalfWidth/
kRoomHalfHeight` **一致** —— 因为 Nav2 用的 `maps/room.pgm` 就是照那两个常量生成的。
改房间尺寸时**三处一起改**，否则机器人在 Gazebo 里会照着错地图撞墙。
