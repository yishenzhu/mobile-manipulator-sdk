# 把 Walle 接到 Robot SDK 上 —— 自然语言驱动机器人（集成计划）

> **这份文档回答一个问题**：怎么把 **Walle**（自研 Agent 框架）和 **robot_sdk** 结合，让 Agent 用自然语言驱动机器人。
>
> - 它是**计划**，不是完成报告。写进来的判断都带依据，但**尚未有端到端实测**。
> - 纪律照旧：**【量】= 本机跑过/读过源码，有依据；【推】= 从代码或框架语义推导，没实测**。
> - Walle 源码：`~/walle`（github.com/yishenzhu/walle）。Robot SDK：本仓库。
>
> **和另外几份的分工**
> - [`file-map.md`](file-map.md) —— 哪个文件装了什么（**要改 SDK 代码时先看它**）
> - [`robot-flows.md`](robot-flows.md) —— 机器人怎么动起来的（mock / Gazebo / 真机三条路）
> - [`sdk-call-flow.md`](sdk-call-flow.md) —— SDK 怎么被调用、`spin()` 在干什么
> - **本文** —— **怎么把 Agent 接进来驱动 SDK**（工具契约 / 两侧配置 / 并发与安全模型 / 里程碑）

---

## 0. 一句话 + 一张图

**一句话**：SDK 是**遥控器**，Walle 是**会思考的驾驶员**；我们给驾驶员配一个**翻译器**（MCP server），把"开到 (1.5, 0)"翻成 SDK 的四类调用，其余（审批 / 超时 / 沙箱 / 可观测 / 评测）全部复用 Walle 现成机制。

```
        用户自然语言："报告电量，然后开到 (1.5, 0.5)，到了告诉我实际位姿"
                              │
                              ▼
        ┌──────────────────────────────────────────┐
        │  Walle（Agent 框架，~/walle）             │
        │  Runner(ReAct) → ToolExecutor(并发·超时)  │
        │  ├─ 审批扩展 allow/deny/ask               │
        │  ├─ 技能 robot-operating（操作 playbook） │
        │  └─ Agent: robot_operator（收窄工具面）   │
        └───────────────┬──────────────────────────┘
                        │  MCP（stdio），工具名 mcp_robot_*
                        │  ← 契约就在这里：工具名 / 参数 schema / 返回 envelope
        ┌───────────────┴──────────────────────────┐
        │  robot_sdk_mcp（本仓库第 11 个包）        │  ← 【翻译器】只翻译，不做策略
        │  持 Robot + spin() 线程；阻塞调用丢线程    │
        └───────────────┬──────────────────────────┘
                        │  robot_sdk（pybind11 绑定）【量】
                        │
        ┌───────────────┴──────────────────────────┐
        │  mock_robot_node / Nav2+MoveIt / 真机     │  ← 换后端不改 Agent（§5.4）
        └──────────────────────────────────────────┘
```

---

## 1. 目标与边界

**要达成的**

- Agent 用自然语言完成："看状态 / 查位姿 / 看障碍 → 导航 / 动机械臂 / 开合夹爪 → 收结果"。
- **换后端不改 Agent**：同一套工具、同一个 `robot_operator`，对 mock 跑通后可原样对 Nav2 + MoveIt 跑。
- 复用 Walle 的治理能力：审批（危险动作人工确认）、工具超时、沙箱（不影响机器人进程）、OTel 可观测、eval 回归。

**不做的（至少第一版）**

- 不让 Agent 直接写 `rclpy` 或裸跑 `ros2` 命令当作主路径 —— 主路径走 MCP 工具（§5.1 会说明为什么这样反而绕开了沙箱纠缠）。
- 不做地图/SLAM 层面的语义理解（"客厅"→坐标）；第一版目标点是数值坐标。
- 不在 bridge 里做动作队列/重试策略（§6 论证为什么）。

---

## 2. 两侧改动总览

| 侧 | 位置 | 改动 |
|---|---|---|
| **SDK** | `src/robot_sdk_mcp/`（**新建，第 11 个包**） | 一个 stdio MCP server：`Robot` 生命周期 + 11 个工具 + 结构化错误翻译 |
| SDK | `docs/agent-integration-plan.md` | 本文 |
| **Walle** | `~/walle/.agent/mcp.yaml` | 注册 `robot` 这个 MCP server（**零改动 Walle 核心代码**） |
| Walle | `~/walle/.agent/agents/robot_operator.md` | 专职 Agent：低温度 + 工具面收窄到 `mcp_robot_*`（用 `walle-cli --agent robot_operator` 选它，见 [`docs/agent-usage.md`](agent-usage.md)） |
| Walle | `~/walle/.agent/skills/robot-operating/SKILL.md` | 操作 playbook（先看状态、运动前预检、失败码处理） |
| Walle | `~/walle/conf.yaml` | 审批规则 + 工具超时覆盖 |

> **核心设计：翻译器保持薄。** `robot_sdk_mcp` 只做"参数校验 → 调 SDK → 翻译结果/错误"，**不做**审批、不做重试、不做并发策略。策略属于上层（Walle 的审批扩展 / 技能 / Agent 定义），这层越薄，越不容易成为新的漂移源。

---

## 3. 能力出口：`robot_sdk_mcp`

### 3.1 工具契约（Walle 侧看到的名字带 `mcp_robot_` 前缀）

> **【量】** Walle 的 MCP 客户端把远端工具统一改名为 `mcp_<server>_<tool>`（见 `~/walle/tools/mcp.py` 的 `_prefixed()`）。server 名叫 `robot`，所以工具对 Agent 呈现为 `mcp_robot_*`。

| 工具（server 内名） | SDK 调用 | 主要参数 | 返回 `data` |
|---|---|---|---|
| `get_status` | `status.wait_for_status(t)` / `query_status(t)` | `timeout_s`, `fresh`(bool) | `state_bits, allows_motion, allows_navigation, battery_percentage, error_code, uptime_seconds` |
| `get_pose` | `navigation.current_pose(t)` | `timeout_s` | `x, y, yaw` |
| `joint_states` | `status.wait_for_joint_states(t)` | `timeout_s` | `name[], position[]` |
| `scan_summary` | `perception.wait_for_laser_scan(t)` + `nearest_obstacle_distance(a0,a1)` | `angle_min`, `angle_max`, `timeout_s` | `beam_count, nearest_m, angle_window` |
| `navigate_to` | `navigation.go_to(NavigationGoal)` | `x, y, yaw`, `avoid_obstacles`, `max_speed`, `timeout_s` | `success, final_distance_error, message` |
| `arm_move_joints` | `motion.move_to_joint_positions(MotionGoal)` | `joint_names[]`, `positions[]`, `max_velocity`, `timeout_s` | `success, final_position_error, message` |
| `move_eef_to_pose` | `motion.move_to_pose(PoseGoal)` | `x,y,z,qx,qy,qz,qw`（7 元组）, `any_orientation`, `timeout_s` | `success, final_position_error, final_orientation_error, message` |
| `gripper` | `motion.open_gripper()` / `close_gripper()` / `set_gripper(p)` | `action`∈{open,close,set}, `position` | `success, final_position_error, message` |
| `home` | `motion.home()` | — | `success, final_position_error, message` |
| `named_target` | `motion.move_to_named_target(name)` | `name` | `success, final_position_error, message` |
| `cancel` | `navigation.cancel_all()` + `motion.cancel_all()` | — | `ok` |

### 3.2 返回契约：**统一 envelope，不抛异常**

```jsonc
{ "ok": true,  "code": "OK",  "message": "arrived", "data": { "success": true, "final_distance_error": 0.04 } }
{ "ok": false, "code": "UNAVAILABLE", "message": "action goal was rejected by the server", "data": {} }
```

> ⚠️ **为什么必须返回 envelope、而不是抛异常**：
> - Walle 的 MCP 客户端在 `result.isError` 时**抛 `RuntimeError`**（`~/walle/tools/mcp.py` 的 `make_fn`）。
> - 抛异常 ⇒ Walle 侧看到的是 `Error: ...`（像崩溃），模型**读不到稳定的错误码**，无法据此分支。
> - 返回 envelope ⇒ 模型按 `code` 决策（重试 / 改道 / 放弃）。
>
> 边界翻译规则（【推】，依据 SDK 的错误模型）：
> - `RobotSdkError` 捕获 → `code` 原样透出（`TIMEOUT` / `UNAVAILABLE` / `INVALID_ARGUMENT` / `OUT_OF_RANGE` / …）。
> - SDK 的 `Result.ok()` 是**通信层**；动作自己的 `success` 是**业务层**。两者**都要在 envelope 里分开表达**（`ok`=通信层，`data.success`=业务层）—— 这是 SDK 反复强调的两层语义，别在翻译层把它压成一层。

### 3.3 形态与生命周期

```
src/robot_sdk_mcp/
├── package.xml / setup.py / setup.cfg / resource/robot_sdk_mcp   # ament_python 包
├── robot_sdk_mcp/
│   ├── __init__.py
│   ├── server.py        # FastMCP 装配 + lifespan（建 Robot、起 spin 线程、退出 stop）
│   ├── tools.py         # 11 个 async 工具（薄封装）
│   ├── config.py        # 从环境变量读 action 名 / node 名 / domain
│   └── errors.py        # RobotSdkError → envelope
├── scripts/run_server.sh  # source ROS + 工作空间 → exec 起 server（给 Walle 的 mcp.yaml 用）
└── README.md
```

⚠️ **不能**用 `ros2 run robot_sdk_mcp robot_sdk_mcp_server`：那个入口脚本的 shebang 是**系统 python3**，那里没有 `mcp`。启动一律走 `scripts/run_server.sh`（它用 venv 的 python 跑 `-m robot_sdk_mcp.server`）。

**【量】Robot 由 `lifespan` 建、退出时停；工具经 Context 注入取它**（`robot_of(ctx)` = `ctx.request_context.lifespan_context["robot"]`）⇒ 没有全局变量，也没有"未启动"的运行时分支。`ctx` **不会**出现在工具 schema 里（v1 按真类型识别并把它从 schema 剔除）。

**lifespan 骨架**（【推】，落地按 mcp v1 的 `FastMCP` API 写）：

```python
@asynccontextmanager
async def lifespan(server):
    robot = robot_sdk.Robot(build_config())          # 自建节点 + 执行器
    threading.Thread(target=robot.spin, daemon=True).start()   # spin 放开 GIL，后台转
    try:
        yield {"robot": robot}
    finally:
        robot.stop()
```

> ⚠️ `Robot` 与 `spin()` 线程的生命周期 = **server 进程生命周期**。不要每个工具调用建一次 `Robot`（会重复建节点、重复起执行器）。

---

## 4. Walle 侧接入（全是配置，零改核心）

### 4.1 `.agent/mcp.yaml`

```yaml
robot:
  command: /home/dfg/robot/ws/src/robot_sdk_mcp/scripts/run_server.sh   # 直接跑脚本，见下
  env:
    ROS_DOMAIN_ID: "42"          # 见 §5.5，必须与 mock / Nav2 用同一个值
  enabled: true
```

> ⚠️ **不要**写成 `command: bash` + `args: ["-lc", ...]`：登录 shell 会 source profile，banner 打到 stdout 就**污染 MCP 的 stdio 协议流**。脚本自己有 shebang。
> `env` 是**合并**进默认环境（`{**default, **server.env}`），不会把 `PATH` 冲掉 —— 但默认环境只带一份安全子集，所以 `ROS_DOMAIN_ID` 必须显式写在这里。

### 4.2 `conf.yaml`（追加）

```yaml
tool:
  approval:
    rules:
      - [allow, "mcp_robot_get_status"]        # 只读，放行
      - [allow, "mcp_robot_get_pose"]
      - [allow, "mcp_robot_scan_summary"]
      - [allow, "mcp_robot_joint_states"]
      - [allow, "mcp_robot_cancel"]            # 急停必须永远可用（还无锁，见 §6）
      - [deny,  "mcp_robot_navigate_to(avoid_obstacles=False)"]   # 危险参数直接拒
      - [ask,   "mcp_robot_*"]                 # 其余运动类默认人工确认
  timeout:
    overrides:
      mcp_robot_navigate_to: 90                # 导航常要几十秒
      mcp_robot_arm_move_joints: 60
      mcp_robot_move_eef_to_pose: 60
      mcp_robot_gripper: 30
      mcp_robot_get_status: 8
      mcp_robot_cancel: 10
```

> ⚠️ 审批规则的 pattern 是 `工具名` 或 `工具名(参数glob)`（见 `~/walle/tools/approval.py`）。**规则按顺序取首个命中**，所以 `deny` 的具体规则要写在 `ask` 的通配规则**前面**。

### 4.3 `.agent/agents/robot_operator.md`

```markdown
---
name: robot_operator
description: 机器人操作员：把自然语言指令翻译成 SDK 的导航 / 运动 / 感知调用
temperature: 0.1
tools:
  allow: ["mcp_robot_*"]
  deny: []
skills: ["robot-operating"]
---
你是机器人操作员。纪律：
1) 动手前先读状态（get_status）；2) 运动前先确认可达（scan_summary / 当前位姿）；
3) 发运动命令前先看 state 是否空闲；4) 运动后必须核对真实位姿；5) 拿到 UNAVAILABLE 先判断是不是"忙"。
```

### 4.4 `.agent/skills/robot-operating/SKILL.md`

技能是**渐进披露**的：Walle 只把 `name/description` 常驻 system prompt，模型按需 `read` 全文。内容要点：

- **机器人同一时刻只执行一个运动目标**（导航与机械臂是两个独立通道，可并行；两段导航 / 两个手臂动作不能同时）。
- **发运动命令前**先 `get_status` 看 `state`（`2=NAVIGATING` / `3=MOVING` 表示忙）。
- **收到 `code=UNAVAILABLE`**：先分辨是"忙"还是"链路问题"——`scan_summary`/`get_status` 能通说明链路在，那就是忙，等当前动作结束再重试。
- **典型序列**：`get_status → scan_summary → navigate_to → get_pose`（核对）→ `gripper`。
- **失败码处理**：`INVALID_ARGUMENT` / `OUT_OF_RANGE` 不要重试原样参数；`TIMEOUT` 先 `get_pose` 看它到底走到哪了。

---

## 5. 关键技术判断（都有依据）

### 5.1 MCP server 天然在沙箱外

- **【量】** Walle 的 bwrap 沙箱是**同名覆盖 `bash` 这一个工具**的扩展（`~/walle/tools/sandbox.py` 的 `api.register_tool(...)`，执行体是 `bwrap ... bash -c cmd`）。
- **【量】** MCP server 走的是 `MCPRegistry.connect()` → `stdio_client(...)`，是 Walle 进程**直接 spawn 的子进程**，从不经过 `bash` 工具 ⇒ **bwrap 套不到它头上**。
- **为什么重要**：若被套进 bwrap，`network=False` 的 `--unshare-net` 会**掐死 DDS**；只读根 + 隐藏路径 + `/dev/shm` 也会伤到 DDS 共享内存传输。
- **结论**：这条正好省掉一个大坑。**附带**：若还想让 Agent 的 `bash` 工具也跑 `ros2` 命令，才会重新碰到沙箱问题 —— 第一版靠 MCP 工具，绕开。

### 5.2 server 必须用 ROS 那个 python

- **【量】** `robot_sdk_py/CMakeLists.txt` 用 `find_package(Python3 ...)` 定 ABI，把 `_robot_sdk*.so` 装进 `lib/python3.<minor>/site-packages/robot_sdk`。ROS Jazzy + Ubuntu 24.04 ⇒ **系统 python3.12**。
- pybind11 扩展模块是 ABI 绑定的：换个解释器（3.11 / 3.13 venv）**import 不了**；`rclpy` 也只在系统 python 里。
- **落地**：给 server 建 `python3.12 -m venv --system-site-packages`（既看得见系统 `rclpy`，又能装 `mcp`）；`robot_sdk` 由 launcher 里 `source install/setup.bash` 塞进 `PYTHONPATH`（venv 的 python 也认 `PYTHONPATH`）。**Walle 自己的 venv 与它解耦**。

### 5.3 工具要写成 `async` + 显式 offload（**版本相关，别搞反**）

- **【量】** Walle 锁 `mcp>=1.0,<2`（`~/walle/pyproject.toml`，且注释说明为何不上 2.x）。
- **【量】mcp v1**（Walle 所在版本）`FuncMetadata.call_fn_with_arg_validation`：同步函数**直接在事件循环线程上调用**，没有任何线程池。
- **【量】mcp v2** 则是 `await anyio.to_thread.run_sync(...)`，同步函数被自动 offload。
- ⇒ **"写同步交给框架"只对 v2 成立**；v1 世界里同步工具会把事件循环**钉死几十秒**（server 连 stdin 都读不了，还会撞客户端超时）。
- **正确形态**：`async def` + 显式 `await asyncio.to_thread(...)`：

```python
async def navigate_to(ctx, x, y, yaw=0.0, avoid_obstacles=True, timeout_s=60.0) -> dict:
    api = robot_of(ctx).navigation()

    def _run():
        goal = robot_sdk.NavigationGoal()
        goal.target = robot_sdk.Pose2D(x, y, yaw)
        goal.avoid_obstacles = avoid_obstacles
        goal.timeout_s = timeout_s
        return api.go_to(goal)

    return envelope(await asyncio.to_thread(_run))
```

- **【量】用标准库 `asyncio.to_thread` 而不是 `anyio.to_thread.run_sync`**：语义等价、少一个直接依赖。
- ⚠️ **【量】`tools.py` 不能写 `from __future__ import annotations`**：v1 用 `issubclass(param.annotation, Context)` 按**真类型**认 `ctx`（`tools/base.py:59`，且 `inspect.signature` 没有 `eval_str`），注解一变字符串就 `TypeError` —— 当场起不来。

> **本质**：`async` 不是目的；要求只有一条 —— **阻塞的 SDK 调用不能跑在事件循环线程上**。v1 逼我们自己做 offload，所以工具体必须 `async`。

### 5.4 换后端不改 Agent

- **【量】** `NavigationConfig.action_name` / `MotionConfig.action_name` / `pose_action_name` 都是可写的。
- **【量】** mock 用 `navigate_to_pose` / `move_to_joint_positions` / `move_to_pose`；适配器占带前缀的 `sdk/navigate_to_pose` 等（同名但类型不同的两个 action server 无法共存）。
- ⇒ 选哪个后端是 **bridge（装配根）** 的事：server 读环境变量 `ROBOT_SDK_NAV_ACTION` / `ROBOT_SDK_MOTION_ACTION` / `ROBOT_SDK_POSE_ACTION` 设好即可，**工具 schema 与 Agent 一字不改**。这正是 SDK 接口解耦（对外只依赖 `robot_sdk_interfaces`）在集成层的兑现。
- **【量】已端到端验证**（R9）：整栈 `ros2 launch robot_sdk_nav2 navigation.launch.py`（map_server + mock + nav2_adapter + 4 个 Nav2 节点，**不含 Gazebo**）起来后，**只把 mcp.yaml 里 `ROBOT_SDK_NAV_ACTION` 改成 `sdk/navigate_to_pose`**，同一个 `robot_operator`、同一句指令、技能一字不动 ⇒ Nav2 侧留下实据：`bt_navigator: Begin navigating from (0.00, 0.00) to (1.50, 0.50)` → `controller_server: Received a goal` → `Passing new path to controller` ×4 → `Reached the goal!`。末距离误差 **0.171 m**（裸 mock 直连时是 0.041 m —— 真 Nav2 有自己的控制容差）。
- ⚠️ **【量】与 README 说法不一致的一处**：两栈并存时 `ros2 action list -t` 显示 `/navigate_to_pose` 上**同时挂着两个类型**（`nav2_msgs` 与 `robot_sdk_interfaces`）—— 和"同名不同型无法共存"的说法对不上，**值得单独复核**。无论结论如何，适配器用 `sdk/` 前缀都是必要的（否则客户端按名取会撞类型）。
- **【量】笛卡尔那半也跑过了**（`move_group_mock` + `moveit_adapter`）：`move_eef_to_pose` 两种 `any_orientation` 都成功返回，`final_orientation_error` 也正确回传；假 move_group 侧留下 `按参数回了成功` ×2。**工具与整条翻译链（SDK action → 适配器 → `/move_action`）机制上打通。**
- ⚠️ **【量】但 `move_group_mock` 不规划、不执行，所以它的 `success=true` 是「假成功」**：实测 `final_position_error = 0.5830952` 恰好等于目标向量模长 `√(0.30²+0.50²)`，且 `/joint_states` 里所有手臂关节仍是 `0.0` —— 末端根本没动，那个"误差"算的是原点到目标的距离。**真验证必须用真 `move_group`**（那条链重得多：SRDF / kinematics / joint_limits / 控制器）。
- **【量】agent 自己抓出了这个假成功**：它心算了模长、比对九位有效数字、又查了 `joint_states`，据此判定"假成功"并**拒绝执行后续的"张开夹爪"**（第一步未证实 + 无法核实夹爪状态，贸然张开可能掉落物体）。这说明技能里"`success` 只是后端自报、要用读数核对"那条规则真的在起作用 —— 也说明**这条技能对未知后端是有防御价值的**。
- **【量】真 MoveIt 链已接通（最后一跳）**：起真执行侧 `ros2_control.launch.py`（controller_manager + joint_state_broadcaster + arm_controller，`mock_components` 假硬件）+ `move_group.launch.py`（真规划）+ `moveit_adapter`，用**我们的 MCP 工具** `move_eef_to_pose` 打通：目标 (0.351, 0.124, 0.421)（`base_footprint`）⇒ `success=True`、位置误差 **9.6 mm**，`/joint_states` 显示**六个臂关节全部真的动了**（pan 0.198 / lift 0.83 / elbow 0.469 / wrist −2.204 / pitch −1.515 / roll 2.769）。自然语言那条也通了（`robot_operator`、审批 1 次、位置误差 **1.8 mm**）。
- **【量】项目自带的真栈探针也 PASSED**：`pytest src/robot_sdk_tests/test/test_real_stack.py -s` ⇒ 两个**带姿态**的 FK 目标都 `success=True`（位置 9.0 / 9.1 mm，姿态误差 0.047 / 0.039 在容忍带内）—— `any_orientation=False` 也解得出来，**证明 6-DOF + KDL 是好的**，之前的"不动"纯粹是挂了假 move_group。
- **【量】修掉桥的一个缺口**：笛卡尔目标的**坐标系原先不可配**（SDK 默认 `map`，真 MoveIt 的模型帧是 URDF 根 `base_footprint` ⇒ MoveIt 的 `ResolveConstraintFrames` 解析不了、目标必失败）。新增 `ROBOT_SDK_POSE_FRAME`（+1 单测）。
- **【量】适配器与 SDK 契约不一致 —— 已查明并修掉**：`MotionAPI` 头注释与 `MoveToPose.action` 的字段注释都写明"`any_orientation` 为真时 `final_orientation_error` **恒为 0**（不是没测，是没要求）"，但 `moveit_adapter` **无条件**拿"实际末端姿态"与 goal 里的姿态比 —— 而 `any_orientation=true` 时 goal 里那个姿态是客户随手填的（我们工具默认**单位四元数** `qw=1`），于是报出 **2.77 / 1.926 rad**（= 实际朝向到单位朝向的夹角），**是个根本没人要求的数**。修法：把 `goal->any_orientation` 存下来，只在客户真要求姿态时才计算该字段。修后实测：`any_orientation=true` ⇒ **0.0**（位置误差 7.3 mm，success）；`any_orientation=false` ⇒ 仍是实测量（真栈探针两个 seed 0.036 / 0.036 rad，**PASSED**）。
- **【量】载荷缺单位**（agent 提出的）：两个误差字段没写单位 ⇒ 已把「米 / 弧度」写进 `move_eef_to_pose` 的工具描述（那正是模型看到的 schema）。
- **【量】MoveIt-only 后端上 agent 会卡在"先查状态"**：这条链没有 mock ⇒ 没 `/robot_status`、`/scan`、map TF，`get_status`/`scan_summary`/`get_pose` 全 TIMEOUT，agent 因此**拒绝发运动命令**（纪律正确）。真部署里状态由真机器人提供；只在"只有 MoveIt"的实验环境里需要显式告诉它别查。
- **【量】4-DOF 遗留已清**：把臂做成 6-DOF 后，代码侧（`joint_trajectory_mock` 的关节表、`probe_pose_real` 的 `ARM`）本来就是 6 关节；清掉的是 6 处**注释/描述**里对旧 4 关节的说明（含 walle 技能描述与本文件里写错的 "4-DOF"）。

### 5.5 用 `ROS_DOMAIN_ID` 隔离 ROS 图

- **【量】** `MCPConfig` 带 `env: dict[str,str]`（`~/walle/conf/config.py`），可直接给 server 设 `ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY`。
- **【量】** 工作空间 README 记过一个坑：**同名 action server 有两个时，客户端会收到不属于自己的回应**，某些交错下真该来的也一起丢。
- ⇒ domain id **一处定义、多处使用**（mock 启动、server、Nav2 launch 同一个值）。建议后续固化进一个 `env` 文件，三处都从它取。

### 5.6 **不排队** —— 互斥交给后端 + Agent（见 §6 详论）

- **【量】** 后端（mock）自己就是"**忙时原子拒收**"：`handleGoal` 在 `mutex_` 里 test-and-set，忙则 `return GoalResponse::REJECT`（`mock_robot_node.cpp` 导航/运动、`joint_trajectory_mock.cpp`、`move_group_mock.cpp` 同款）。
- ⇒ bridge **不加锁、不排队**。理由与安全论证见 §6。

### 5.7 启动时要等动作服务端 —— 否则**第一个目标会丢**

- **【量】** rclcpp_action **不会替你等**服务端被发现。不等的话第一个目标就发在发现完成之前，然后一直等不到受理：实测 `home()` 稳定 `TIMEOUT: timed out waiting for the action goal to be accepted`（30 s），而紧接着**同一个调用**成功。
- 前几轮的用例没暴露，是因为它们开局先调了 `joint_states()` / `get_status()`，等于顺手等了约一秒。
- ⇒ `lifespan` 起完 spin 线程后，`navigation().server_available(5s)` + `motion().server_available(5s)`。

---

## 6. 并发与安全模型

**背景（【量】）**：Walle 的 `ToolExecutor.execute_calls` 用 `asyncio.as_completed([...])` **并发**执行同一轮的多个工具调用（`~/walle/core/executor.py:162`）。所以模型一轮里发两个运动工具，它们会同时进 server。

### 6.1 为什么不排队

| 维度 | 排队（bridge 加锁） | 不排队（交给后端 + Agent） |
|---|---|---|
| 正确性 | 后端本来就保证 | 后端本来就保证 ⇒ 锁是纯冗余 |
| 失败模式 | 第二个等到第一个结束，可能**撞上 Walle 的工具超时** ⇒ 模型拿到含糊的 `timed out` | 第二个**立刻**被拒 ⇒ 模型拿到清晰的 `UNAVAILABLE`（可重试） |
| 安全性 | **fail-open**：偷偷执行了模型可能已改主意的动作 | **fail-closed**：什么都不发生 |
| 粒度 | 一把粗锁会把**合法的并发**（导航+机械臂）也串行化 | nav / motion 是两个独立 slot，各自独立 |
| 分层 | 策略渗进"翻译器"，bridge 变厚 | bridge 保持薄，策略留在技能/审批层 |

> **一句话**：**队列让人等，拒收让人知。** 对物理系统，**"不做" 比 "多做" 安全**。

### 6.2 Agent 负责要成立，得给三样东西

1. **明确的结构化错误**：`UNAVAILABLE` + 原文 message，原样透出（§3.2）。
2. **能自查"忙不忙"**：`get_status` 直接给 `allows_motion` / `allows_navigation`（**分通道** —— 旧版单值 `state` 做不到这点：臂在动时底盘其实照常能发）。要自己看就认 `state_bits` 的位。
3. **技能里的规则**：§4.4 写明"同一时刻一个运动目标 / 先看 state / 被拒即忙"。

### 6.3 若日后发现错误码有歧义

`UNAVAILABLE` 同时表示"链路断"和"忙"。若实践中真的撞上歧义，再加一个**只检查、不排队**的守卫：server 维护"某 slot 正在飞行"的标志，第二个请求**立刻返回 `BUSY`**，不等。**先不加** —— 别提前上状态。

### 6.4 急停永远无锁

`cancel` 工具**不拿任何锁**（本来也没有锁）。否则正在跑的导航会把急停挡住 —— 那不是不便，是安全事故。

---

## 7. 验证路径（先 mock，不需要 Gazebo）

1. 终端 1：`ros2 run robot_sdk_sim mock_robot_node`（`ROS_DOMAIN_ID=42`）
2. 单独起一次 server，`list_tools` 应拿到 11 个工具（调试 bridge 本身）：
   `ros2 run robot_sdk_mcp robot_sdk_mcp_server`
3. Walle 起 `robot_operator`，输入：
   > 报告电量，然后开到 (1.5, 0.5)，到了告诉我实际位姿

   断言：调用序列含 `get_status → navigate_to → get_pose`；`ok=true`；末位置与目标差 < 容差。
4. 加一句"张爪"验证 `gripper`；再跑一次 `cancel` 验证急停可用。
5. **并发用例**（验 §6 的取舍）：让它一轮里同时"开过去 + 张爪"。预期：导航成功、手臂动作可能被拒一次 → 模型按技能提示等待重试。
6. **换后端不改 Agent**：起 `robot_sdk_nav2 navigation.launch.py` + MoveIt 适配器，server 设 `ROBOT_SDK_*_ACTION=sdk/...`，**同一个 `robot_operator` 重跑**，工具面与 Agent 定义不变。

---

## 8. 里程碑

| 阶段 | 内容 | 状态 |
|---|---|---|
| **P1** | `robot_sdk_mcp` 包骨架 + 11 个 async 工具 + envelope + launcher/venv | ✅ 完成（venv 装 `mcp 1.30.0`） |
| **P2** | Walle 接入：`mcp.yaml` + `robot_operator` + `robot-operating` 技能 + 审批/超时 | ✅ 完成 |
| **P3** | mock 端到端 + 急停用例 | ✅ 完成（真 CLI 通道：`navigate_to` 审批 1 次；`cancel` 0 次） |
| **P4** | 接 Nav2 整栈（**换后端不改 Agent**） | ✅ 完成（Nav2 侧留下 `Reached the goal!`；只改了 mcp.yaml 一行） |
| **P5** | 可观测 + 评测 | ⏭️ **跳过**（本机无 docker，不做） |
| **P6** | 文档与测试收尾 | ✅ 完成（`robot_sdk_mcp` 单测 9 项；两侧 README / 本文） |

**当前测试基线**：SDK 侧 `colcon test` **88 项 0 失败**；`robot_sdk_mcp` 单测 **9 项 0 失败**。另有一个在 R5c 顺手修掉的 SDK 侧真 bug（见 §9）。

---

## 9. 风险与未决

- **mcp 版本混搭**：server 侧已钉 `mcp>=1.0,<2`（`setup.py` 的 `install_requires` + venv 里实际装的 1.30.0），与 Walle 同大版本 ⇒ 不依赖 v2 的自动 offload。**跨大版本混搭仍未验证**。
- **【推】并发进 SDK 的额外安全**：§5.6 说后端能拒收，但"两个来自不同 server 实例 / 有人手动发"的情况超出 bridge 管辖，仍落到后端。**未压测**。
- **`nearest_obstacle_distance` 的 `inf`**：**已处理** —— `scan_summary` 映射成 `nearest_m=null`，不让模型看到 `Infinity`。
- **【量】SDK 侧顺手修掉的一个真 bug（R5c）**：`ActionHandle` 与 rclcpp_action 的 GoalHandle 构成**引用环**（反馈回调捕获 `[handle]`，而 GoalHandle 又被 `goal_future_` 持有）⇒ 句柄永不析构、`active_handles_` 里的 weak_ptr 永不失效 ⇒ `cancelAll()` 拿着一个**早已结束**的目标去取消，client 抛 `Goal handle is not known to this client`。修法极小：`wait()` 在**请求过结果的两条出口**上都 `goal_future_ = {}`（对应 ROS 收到结果就 erase 自己的登记）。改前三种场景两种会抛、改后全不抛；`colcon test` 88 tests / 0 failures（C++ 改动共 7 增 1 删，一个文件）。
- **未决**：地图语义（"客厅"→坐标）不在第一版；`TransformManager` 未绑定到 Python（SDK 的 `tf()` 没暴露），若需要姿态查询要单独加绑定。
- **Walle 侧无源码改动**：所有接入都在 `.agent/` 与 `conf.yaml`。若日后要更强的机器人语义（如自动重规划），再考虑加 Walle 扩展（`.agent/extensions/`）。
