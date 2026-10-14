# Changelog

记录**契约面**的变化（ROS 接口 / C++ 公开头 / Python 面 / MCP 工具面 / 配置面）。
格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，版本号遵循[语义化版本](https://semver.org/lang/zh-CN/)。
"只增不删"的政策、什么算破坏、发版步骤：见 [`docs/contract.md`](docs/contract.md)。

## [Unreleased]

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
