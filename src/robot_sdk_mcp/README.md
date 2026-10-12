# robot_sdk_mcp

把 Robot SDK 的四类能力（导航 / 运动 / 感知 / 状态）暴露成 **MCP 工具**，供 Agent 框架
（Walle）以自然语言驱动机器人。**只做翻译**：参数校验 → 调 SDK → 翻译结果与错误；
审批 / 重试 / 并发策略都在上层（Agent 的审批扩展、技能、Agent 定义）。

设计与踩坑见 [`../../docs/agent-integration-plan.md`](../../docs/agent-integration-plan.md)。

## 准备（一次）

要一个**带 `mcp` 的 venv**，且必须能看见系统的 `rclpy`（所以带 `--system-site-packages`）：

```bash
cd src/robot_sdk_mcp
python3 -m venv --system-site-packages .venv
.venv/bin/pip install "mcp<2"        # 与 Walle 同大版本
```

> ⚠️ **不要用 `ros2 run robot_sdk_mcp robot_sdk_mcp_server`**：那个入口脚本的 shebang 是
> 系统 python3，那里没有 `mcp`。启动一律走 `scripts/run_server.sh`（它用 venv 的 python）。

## 跑

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash   # 在 ws 根目录
bash src/robot_sdk_mcp/scripts/run_server.sh                    # 单独起（stdio，调试用）
```

接 Agent 时由 Walle 拉起（`.agent/mcp.yaml`）：

```yaml
robot:
  command: /home/dfg/robot/ws/src/robot_sdk_mcp/scripts/run_server.sh   # 直接跑脚本
  env: { ROS_DOMAIN_ID: "42" }   # 必须与 mock / Nav2 用同一个值
  enabled: true
```

> ⚠️ 不要写成 `command: bash` + `args: ["-lc", ...]`：登录 shell 会 source profile，
> banner 打到 stdout 就污染 MCP 的 stdio 协议流。脚本自己有 shebang。
> （`env` 是**合并**进默认环境，不会把 PATH 等冲掉 —— 见 mcp 的 `stdio_client`。）

## 环境变量

| 变量 | 作用 | 默认 |
|---|---|---|
| `ROBOT_SDK_NODE_NAME` | SDK 节点名 | `robot_sdk_mcp` |
| `ROBOT_SDK_NAV_ACTION` | 导航 action 名（接适配器则换 `sdk/navigate_to_pose`） | `navigate_to_pose` |
| `ROBOT_SDK_MOTION_ACTION` | 关节 action 名 | `move_to_joint_positions` |
| `ROBOT_SDK_POSE_ACTION` | 笛卡尔 action 名 | `move_to_pose` |
| `ROBOT_SDK_GLOBAL_FRAME` | 全局坐标系 | `map` |
| `ROBOT_SDK_USE_SIM_TIME` | 用仿真时间 | `false` |
| `ROBOT_SDK_JOINT_NAMES` | 逗号分隔；具名目标要靠它把位置对应到关节 | 空 |
| `ROBOT_SDK_HOME_POSITIONS` | 逗号分隔；配出 `named_targets["home"]` | 空 |
| `ROS_DOMAIN_ID` | 隔离 ROS 图，两侧（server 与 mock）必须一致 | rcl 默认 |

## 工具（11 个）

| 工具 | SDK 调用 | 说明 |
|---|---|---|
| `get_status` | `status.wait_for_status` / `query_status` | `fresh=true` 走服务查询 |
| `get_pose` | `navigation.current_pose` | 全局坐标系下的 x / y / yaw |
| `joint_states` | `status.wait_for_joint_states` | 关节名与位置 |
| `scan_summary` | `perception.wait_for_laser_scan` + `nearest_obstacle_distance` | 窗口内最近障碍；无回波 `nearest_m=null` |
| `navigate_to` | `navigation.go_to` | 世界坐标 (x, y, yaw) |
| `arm_move_joints` | `motion.move_to_joint_positions` | 后端按关节名对齐 |
| `move_eef_to_pose` | `motion.move_to_pose` | 需接 MoveIt 适配器（裸 mock 没有这个 action） |
| `gripper` | `open_gripper` / `close_gripper` / `set_gripper` | `action=open/close/set` |
| `home` | `motion.home` | 依赖配置里的具名目标 `home` |
| `named_target` | `motion.move_to_named_target` | 同上 |
| `cancel` | `navigation/motion.cancel_all` | 急停；各自尽力而为，失败记在 `failures` |

统一返回 `{"ok", "code", "message", "data"}`：`ok` 是**通信层**，`data.success` 才是**业务层**。
失败不抛异常，模型可按 `code` 分支（`TIMEOUT` / `UNAVAILABLE` / `INVALID_ARGUMENT` / …）。

## 四条约定（别改坏）

1. 工具体一律 `async` + `asyncio.to_thread`：SDK 的阻塞调用跑在事件循环上会**卡死 server**。
   本包钉 `mcp<2`，而 **v1 里同步函数是直接在事件循环上执行的**。
2. `tools.py` **不能**写 `from __future__ import annotations`：v1 用
   `issubclass(param.annotation, Context)` 按**真类型**认 `ctx`，注解成字符串就 `TypeError`。
3. `Robot` 在 `server.py` 的 `lifespan` 里建、退出时停；工具经 `Context` 取（`robot_of(ctx)`），
   所以 `ctx` 不会出现在工具的 schema 里。
4. 启动时等服务端就位（各 5 s）：**rclcpp_action 不替你等**，不等的话第一个目标会丢。
