# Changelog

记录**契约面**的变化（ROS 接口 / C++ 公开头 / Python 面 / MCP 工具面 / 配置面）。
格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，版本号遵循[语义化版本](https://semver.org/lang/zh-CN/)。
"只增不删"的政策、什么算破坏、发版步骤：见 [`docs/contract.md`](docs/contract.md)。

## [Unreleased]

### 加了（新能力）

- **唯一性守卫**：mock 默认只允许「本机 + 本域」跑一个实例（`$XDG_RUNTIME_DIR` 下的 `flock` 锁文件；
  锁打不开即拒绝启动，退出码 1）。同域要并存加 `--ros-args -p unique:=false`。边界与取舍见 `docs/improvements.md`。
- mock 发布 `/robot_description`（latch），并把轮子关节并入 `/joint_states` ⇒ RViz 里轮子/底盘的 TF 完整。

### 修了（缺陷修复）

- mock 的到位判据原先只看 xy、朝向又直接赋值 ⇒ **原地转身在 RViz 上是瞬移**；现改为限速转向 + 位姿共同判定。
- mock 在 `remaining` 为 0（原地转身）时除零 ⇒ NaN 写进位姿，此后 **所有 TF 被 RViz 丢弃**（模型/轮子/深度图全红）；已加除零保护。
- 可视化：RViz 缺 TF（相机/激光/底盘链）、`black` 材质重复定义、MockRobotModel 默认未启用等一串问题修复。
- **传感器帧重复发布**：mock 与 URDF 各发一套挂载 TF ⇒ 同一个 `imu_link`/`camera_link` 有两个父节点、
  `TF_REPEATED_DATA`、位置在两个值之间跳；已统一到 URDF（`/tf_static` 发布者 2 → 1）。

### 变了（行为变化）

- **超时错误更精确**：`wait_for_*` 超时时先查原因 —— 有发布端但 QoS 不兼容、或话题上没有任何发布端时，
  改报 `UNAVAILABLE` 并附具体原因（以前一律是 `TIMEOUT`）。查不出别的原因时，措辞与以前**一字不差**。
  实现见 `detail/subscription_wait.hpp`，背景见 `docs/qos.md` §6。

- **⚠️ 破坏性：到位容差移出 SDK 配置面**。`navigation.goal_tolerance_m` → `limits.goal_tolerance_m`；
  新增 `limits.{yaw_tolerance_rad,max_speed,max_yaw_rate,joint_tolerance,default_joint_velocity,body_clearance_m}`。
  它们是**机器人的属性**（由执行侧使用），SDK 客户端不读 —— 见 `config.hpp` 的 `RobotLimitsConfig`。
- **导航结果会被核实**：`navigateTo()` 拿到 `success=true` 后会读回位姿核对，超差即把 `success` 判为 false
  （阈值 `navigation.arrival_tolerance_m` / `arrival_yaw_tolerance_rad`，默认**比后端容差宽**，避免误判正常结果）；
  **读不回位姿时不改判**（不因查询失败把成功的动作判死）。
- **mock 按能力上限限速**：客户给的 `max_speed` 超过 `limits.max_speed` 时按上限执行并打 WARN（以前照单全收）。
- arm 换用官方 UR5 模型（真实运动学与网格，保留原 link/joint 名），并加 0.5m 底座避免零位自碰。
- **⚠️ 破坏性：`/scan` 的 `frame_id` 由 `laser_frame` 改为 `laser_link`**（统一到 URDF 的帧名；
  按帧名投影的消费方需要同步）。
- **传感器挂载点以 URDF 为准**：mock 不再自己广播激光/IMU/相机的挂载 TF（改由
  `robot_state_publisher` 从 URDF 发到 `/tf_static`），mock 需要的挂载量改为**从 URDF 读**。
  连带：相机挂载高度 0.3 → 0.11、激光 x 0.1 → URDF 值（与模型一致了）。URDF 补了
  `camera_optical_frame`（深度图必须在光学系里）。⚠️ 裸跑 mock 将没有传感器 TF。

## [0.1.0]

首个版本：把"自然语言驱动机器人"这条链路跑通，并把 SDK 的契约面定下来（**此后只增不删**）。
尚未打 tag（步骤见 `docs/contract.md`）。

### 公开契约

- **ROS 接口**：`RobotStatus`（msg）+ `MoveToJointPositions` / `MoveToPose` / `NavigateToPose`（action）
- **C++ 公开头**：`robot_sdk/*.hpp`（`robot_sdk.hpp`、`robot.hpp`、四个能力 API、`config.hpp`、
  `result.hpp`、`error.hpp`、`types.hpp`、`action_handle.hpp`、`transform.hpp`、`version.hpp`）
- **Python 面**：`robot_sdk.Robot` + 四个能力 API + `RobotSdkError` / `ErrorCode`
- **MCP 工具面**：11 个工具（`get_status`、`get_pose`、`joint_states`、`scan_summary`、`navigate_to`、
  `arm_move_joints`、`move_eef_to_pose`、`gripper`、`cancel`、`home`、`named_target`），统一信封 `{ok, code, message, data}`
- **配置面**：ROS 参数名（含 `qos.*`）与环境变量名（`ROBOT_SDK_*`）

### 值得记一笔的实现决定

- C++ 侧**不抛异常**：运行期接口一律返回 `Result`；只有构造期（没有 `Result` 通道）例外，
  由绑定层翻成 `RobotSdkError(INVALID_ARGUMENT)`。
- **不排队**：忙就拒（后端原子 test-and-set），互斥交给"后端拒收 + 状态可读 + 技能规则"。
- QoS 集中在 `QosConfig`：一处配好、各模块按需取用；`reliability` 保守用字符串（ROS 参数没有枚举类型）。
- 边界约定：Python 绑定在会阻塞的调用上放开 GIL（`wait_*`、动作、`spin`）。

### 已知边界

- 真机适配未做；Gazebo 那条链路依赖物理精度，**故意不进 `colcon test`**。
- `move_group` 侧打通到 6 DOF 规划，执行反馈未调优。
- 未做：并发扩缩容基准（多线程各自读缓存是否被 GIL 串住）。
