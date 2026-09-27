# robot_sdk_tests

跨包的**验收探针**：验的是**行为**（忙时拒收、取消、折角、避障预检、加速度生效），
不是单个函数 —— 那些在 `robot_sdk_core` 的单元测试里。

放在仓库里的意义是**可复核**：谁都能跑一遍，不用再靠临时写一段。

## 每个探针自己会判定

它们会拉对象、发目标、看结果，最后打印 **✅ / ❌** 并用**退出码**说话 ——
失败了不会假报成功（`console_scripts` 不看返回值，所以每个探针都走 `cli()` 把返回值转成退出码）。

| 探针 | 验什么 | 需要谁在跑 |
|---|---|---|
| `probe_motion` | 忙时拒收 + 取消 | mock 机器人 **或** 适配器 |
| `probe_wrap` | 连续关节走最短的一边 | 同上，且对方**声明了** `shoulder_pan_joint` 是连续关节 |
| `probe_avoid` | `avoid_obstacles` 的两种取值 | mock 机器人 |
| `probe_accel` | `max_acceleration` 真的生效 | mock 机器人 **或** 适配器 |

## 打 mock 机器人（最省事）

```bash
ros2 run robot_sdk_sim mock_robot_node
```

另一个终端：

```bash
ros2 run robot_sdk_tests probe_motion move_to_joint_positions
ros2 run robot_sdk_tests probe_avoid
ros2 run robot_sdk_tests probe_accel
```

## 打适配器 + 假下游

```bash
ros2 launch robot_sdk_tests motion_chain.launch.py
```

另一个终端：

```bash
ros2 run robot_sdk_tests probe_motion
ros2 run robot_sdk_tests probe_accel /sdk/move_to_joint_positions
```

## `probe_wrap` 还要多一步

它验的是"连续关节走最短的一边"，所以**得先声明哪个关节是连续的** ——
不声明的话它走 6.7 rad（绕一整圈），探针会**如实报失败**：

```bash
# 对 mock：
ros2 run robot_sdk_sim mock_robot_node \
  --ros-args -p 'motion.continuous_joints:=[shoulder_pan_joint]'

# 对适配器：
ros2 run robot_sdk_bridge motion_adapter \
  --ros-args -p 'continuous_joints:=[shoulder_pan_joint]'
```

然后：

```bash
ros2 run robot_sdk_tests probe_wrap            # 打 mock
ros2 run robot_sdk_tests probe_wrap /sdk/move_to_joint_positions   # 打适配器
```
