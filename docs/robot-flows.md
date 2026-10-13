# 机器人是怎么动起来的 —— 业务流转全图（mock / Gazebo / 真机）

> **这份文档回答一个问题**：从客户敲下一行代码，到机器人真的动起来，中间都发生了什么？
>
> - 想知道 **mock 内部为什么这么设计**（三态槽位、一把粗锁的论证、凭证式加锁）→ `mock-robot-node.md`
> - 想知道 **SDK 怎么用、`spin()` 到底在干什么**（偏入门、图更多）→ `sdk-call-flow.md`
> - **本文**是**把两边接起来的那张图**，并且把 **mock / Gazebo / 真机** 三条路并排摆开对照。
>
> 纪律照旧：**【量】= 本机跑过，有数字；【推】= 从代码/框架语义推导，没实测**。

---

## 0. 一句话 + 一张图

**一句话**：SDK 是**遥控器**，契约是**插头标准**，mock / Gazebo / 真机是**三种可以被插上的电器**。换电器不用换遥控器。

```
        客户代码（robot_sdk 的头文件）
             │
             ▼
    ┌────────────────────┐
    │  robot_sdk (core)  │   ← 【客户端】只发目标、收结果。它**不是机器人**
    │  Node + Executor   │
    └─────────┬──────────┘
              │   DDS（话题 / 服务 / action）
              │   ← 契约就在这里：名字、消息类型、帧名、容差
    ┌─────────┴──────────────────────────────┐
    │                                        │
    ▼                                        ▼
┌─────────────────┐              ┌───────────────────────────┐
│ mock_robot_node │   或者        │ Gazebo / 真机              │
│  （机器人端点）  │              │  （第 6 阶段那侧）          │
└─────────────────┘              └───────────────────────────┘
```

**三条边界**（看懂这三条，后面全通）：

| 角色 | 是什么 | 它**不做**什么 |
|---|---|---|
| **客户代码** | 你的程序。只 include `robot_sdk` 的头 | 不直接碰 ROS 的话题/动作 |
| **robot_sdk** | 一个**客户端库**。建节点、发目标、收结果、查 TF | **它不是机器人**，机器人不在这个进程里 |
| **执行侧** | mock / Gazebo / 真机。**它才是那个"机器人"** | 不认识"客户"是谁，只认 DDS 上的目标 |

> 判据（全项目通用）：*换掉后端（Gazebo/真机）之后这个配置还有意义吗？* 有 → 进 `RobotConfig`（=契约）；没有 → 留在 mock 自己手里（见 `mock-robot-node.md` §7.1）。

---

## 1. 一次"去某个位置"的完整旅程

这是**本文的主线**。客户调一句 `goTo(goal)`，下面这 8 步按时间顺序发生。

```
时间轴 ──────────────────────────────────────────────────────────────▶

① 启动（一次）        ② 发目标（一次）      ③ 每 20 ms（一直）      ④ 收尾（一次）
客户线程              客户线程              mock 控制回路线程       客户线程
Robot(config)         goTo()               controlTick()          handle->wait() 醒来
  ├ 建节点              └ 发 action goal      └ navTick 推进一步       └ 判定 + 抛异常
  ├ 建多线程执行器             │                    │
  └ spin()（丢后台线程）        │                    │
                          DDS ─┴─▶ mock 动作车道    │
                          handleNavGoal → 写意图 ────┘
```

### ① 启动：`Robot(config)` 干了什么

| 步骤 | 做什么 | 在哪 |
|---|---|---|
| 建节点 | `rclcpp::Node(node_name, node_namespace)`。名字和命名空间是**两个正交参数** | `robot_sdk_core/src/runtime.cpp:17` |
| 建执行器 | `MultiThreadedExecutor`（**多线程**，不是单线程 spin） | `runtime.cpp:20-21` |
| 分三条车道 | `control_group_`（互斥，导航+运动共用）、`sensor_group_`（可重入，感知）、`status_group_`（互斥，状态） | `runtime.cpp:31-39` |
| 装配四个 API | `NavigationAPI` / `MotionAPI` / `PerceptionAPI` / `StatusAPI` 各拿自己那份配置 | `robot_sdk_core/src/robot.cpp:19-27` |

**`spin()` 必须调。** 它不是"派活"，而是**让工人（线程池）上班**。不调的话：

| 你以为 | 实际 |
|---|---|
| "忘了 spin 只是慢一点 / 会超时重试" | ❌ **完全收不到** —— 目标回应、进度、结果、服务应答、话题消息，全都要靠执行器把回调跑起来 |

> 示例代码都写成 `std::thread spinner([&robot]{ robot.spin(); });`：`spin()` 会阻塞到 `stop()`，所以必须扔到后台，主线程留给阻塞式 API。

### ② 发目标：一句 `goTo()` 变成什么

| SDK 方法 | 发到哪 | 名字从哪来 |
|---|---|---|
| `navigation().goTo(goal)` / `goToAsync` | action `robot_sdk_interfaces/NavigateToPose` | `config.navigation.action_name`（默认 `navigate_to_pose`）|
| `motion().moveToJointPositions(...)` | action `.../MoveToJointPositions` | `config.motion.action_name`（默认 `move_to_joint_positions`）|
| `motion().home()` / `moveToNamedTarget("home")` | 先查 `named_targets` 折成关节目标，再走上面那条 | `config.motion.named_targets` |
| `status().queryStatus()` | **服务** `GetRobotStatus` | `config.status.get_status_service` |
| `status()` / `waitForStatus` | **话题** `RobotStatus` | `config.status.status_topic` |
| `perception().laserScan()` / `waitForLaserScan` | **话题** `LaserScan` | `scan` |
| `navigation().currentPose()` | **TF 查询**（不是话题） | `global_frame`(map) → `base_frame`(base_footprint) |

**重点**：所有名字都是**相对名**（`"scan"`、`"navigate_to_pose"`），由 rcl 按节点命名空间展开。所以一份代码 + 不同 `node_namespace` = 多个互不干扰的机器人。**SDK 不重复实现命名空间机制**，那是 ROS 2 内建的。

### ③ mock 收到目标：**先写意图，不动手**

```
客户 ──goal──▶ DDS ──▶ 执行器 ──▶ 【动作回调车道】
                                    handleNavGoal:
                                    ├─ 拿 mutex_
                                    ├─ 已经在导航？ → 拒绝（可观测！见下）
                                    ├─ nav_.slot.reserve()        ← 写"意图"：占位
                                    ├─ nav_.target = (2.0, 1.0)   ← 写"意图"：去哪儿
                                    ├─ nav_.max_speed = 0.5       ← 写"意图"：多快
                                    └─ 放 mutex_
                                    ↑ 机器人**一动没动**，pose_ 还是 (0,0)

框架紧接着（同一线程）──▶ handleNavAccepted:
                                    └─ nav_.slot.begin(handle)    ← 相位 → kRunning + 句柄就位
```

**「拒绝」而不是「排队」**：拒绝是**可观测**的（客户拿到 `UNAVAILABLE: action goal was rejected by the server`），排队是**静默**的（客户等到超时都不知道为什么）。宁可让客户立刻知道。

### ④ 每 20 ms：**只有定时器改真值**

```
【控制回路车道】每 20 ms：
controlTick():
├─ cmdVelTick()      ← 底盘命令（Nav2 那条，见 §3.4）
├─ navTick()         ← 朝目标走一步，改 pose_        ← 【改真值】
├─ motionTick()      ← 关节走一步，改 joint_positions_ ← 【改真值】
├─ stamp = now()     ← 一拍**只取一次**时间
├─ publishJoints(stamp)   50 Hz
├─ publishFrames(stamp)   50 Hz（map→odom→base_footprint）
└─ publishSensors(stamp)  内部降频到 10 Hz
```

到位了就返回结果：`pose_` 直接对齐到目标（不留毫米级残差），`success=true`、`final_distance_error`、`message="reached target"`。中途每 5 拍发一条 feedback（`kFeedbackEveryNTicks=5` → 20 Hz）。

> **为什么不能让回调推进机器人？** 因为两者的时间特性完全不同：
> 回调是**事件驱动**（客户晚 300 ms 发目标，第一步就走得晚 300 ms → 模拟出的速度会随网络抖动变化）；
> 定时器是**固定 20 ms**。
> 所以：**回调只决定"收不收、收什么"，定时器负责"每 20 ms 走一步"。**

### ⑤ 回到客户：**两层语义，务必分清**

```cpp
auto handle = robot.navigation().goToAsync(goal);
auto wrapped = handle->wait();          // ← 第①层：通信层
if (!wrapped.ok()) { /* Error：超时 / 服务端不在 / 被取消 */ }
if (!wrapped.value().success) { /* 第②层：业务层 —— 它到了吗 */ }
```

| 层 | 字段 | 失败长什么样 | 谁填 |
|---|---|---|---|
| **通信层** | `Result<T>::ok()` / `error()` | `Error`，码是 `kTimeout` / `kUnavailable` / `kCancelled` / `kInternal` | SDK 自己（发送失败、等待超时、终态码不是 SUCCEEDED 时用 `resultCodeToString` 补 message） |
| **业务层** | `result.success` / `final_*_error` / `message` | 只是 `success=false` + 一句人话 | **执行侧** |

> ⚠️ SDK 的 action 定义里**没有** `error_code` 数字字段，只有 `success` + 一个误差值 + `message`。**数字型 `error_code` 出现在下游**（`RobotStatus`、Nav2、MoveIt），由**适配器**折算成 `success`+`message` 再报上来（§5）。

`Result::value()` 在失败时抛 `SdkException` —— 它是**安全网，不是常规通道**（常规走 `ok()/error()`）。

### ⑥ Python 侧同一张图

| C++ | Python | 差别 |
|---|---|---|
| `Result` 有 `ok()` | `unwrap()`：失败直接 `throw RobotSdkError` | Python 里所有失败只剩 `RobotSdkError` |
| 可选值返回 `Result` | `projectOptional()`：没有就返回 `None`（**不抛**） | "现在有吗"不是错误 |
| 阻塞调用占线程 | 包在 `py::gil_scoped_release` 里（**放 GIL**） | 否则别的 Python 线程全卡住 |
| `Robot::spin()` | **同样放 GIL** | 不放会死锁：后台 spin 线程转不动，所有阻塞 API 一起挂 |
| 构造期 rclcpp 异常 | 翻译成 `RobotSdkError`（否则漏成 `ValueError`） | `robot_sdk_bindings.cpp` 那个 try/catch |

---

## 2. mock 到底提供了哪些"业务"

| 客户的需求 | 形态 | 为什么是这种形态 | 频率 |
|---|---|---|---|
| "机器人现在怎么样？"（不阻塞，只关心最新值） | 话题 `RobotStatus` | 话题是**推送**语义：客户不用问、也不用等 | 10 Hz |
| "**此刻**确切的状态"（同步、要能拿到关节） | 服务 `GetRobotStatus` | 话题只有"最新值"，**没有"我问的那一刻"** | 按需 |
| "关节现在在哪？" | 话题 `JointState` | 50 Hz × N 个关节塞进状态话题会把状态话题变重 | 50 Hz |
| 坐标变换 | TF：动态两条 + 静态四条 | TF 是 ROS 2 的基础设施。**不广播 TF 的机器人等于不存在** | 50 Hz / 一次 |
| "去某个位置" | action `NavigateToPose` | 长耗时、要进度、要能取消 → 只有 action 合适 | 按需 |
| "动关节 / 开关夹爪" | action `MoveToJointPositions` | 同上 | 按需 |
| "周围有什么？" | 5 条传感器话题 | 传感器是**持续流**，客户要"最新一帧"而不是排队 | 10 Hz |
| （**多听**的一样）底盘速度 | 订阅 `cmd_vel` | 让 Nav2 这类"出口就是 cmd_vel"的栈直接开车 | 事件 |

**两个"为什么不是别的"：**

- **状态为什么既发话题又开服务？** 它们回答的是**不同的问题**："大概现在怎么样"（推送、可能丢帧、够用）vs"此刻到底怎么样"（同步、确切）。这不是冗余 —— 冗余是**同一份数据存两处**，而那正是 mock 要消灭的：状态话题和服务**共用同一个 `makeStatus()`**，因为曾经踩过"话题说 A、服务说 B"。
- **`cmd_vel` 为什么不算契约？** 客户从不发 `cmd_vel`，它是**机器人内部的总线**。所以话题名由 mock 自己的参数 `cmd_vel_topic` 决定，**不进 `RobotConfig`**。

**mock 刻意不做的事**（功能缺口，不是 bug）：

| 缺口 | 现状 | 理由 |
|---|---|---|
| 不做 TF 变换 | 导航目标帧必须是 `map`，否则**拒绝** | 那是导航后端的活 |
| 不会绕路 | `avoid_obstacles=true` 时**预检**"目标走得到吗"，走不到当场报失败；`false` 则直冲 | 绕路＝在 mock 里重新发明导航栈 |
| 同类目标一次只跑一个 | 第二个目标**拒绝** | 真实机器人也不会同时执行两条导航 |
| 没有传感器噪声 | 五路都在发，但都是**理想值** | 加噪声会让那张"解析解"验收表失去意义 |

---

## 3. 四条车道 + 一把锁（通俗版）

mock 同时被**四种**东西驱动，它们跑在**不同线程**上：

```
        ┌──────────────────────────────────────────────────────┐
        │              MultiThreadedExecutor                    │
        └───────┬──────────────────┬────────────────┬──────────┘
  客户请求到达│       定时器到点  │                │收到 cmd_vel
                ▼                  ▼                ▼
      ┌─────────────────┐  ┌──────────────┐  ┌──────────────────┐
      │  动作回调车道    │  │  控制/状态回路 │  │  服务回调车道     │
      │ handleNavGoal   │  │ controlTick  │  │                  │
      │ handleNavCancel │  │   50 Hz      │  │ status_service   │
      │ handleMotion*   │  │ statusTick   │  │                  │
      │                 │  │   10 Hz      │  │                  │
      └────────┬────────┘  └──────┬───────┘  └────────┬─────────┘
               │                  │                   │
               │ 写"意图"          │ 写"真值"           │ 只读
               ▼                  ▼                   ▼
        ┌─────────────────────────────────────────────────┐
        │              共享状态（mutex_ 保护）              │
        └─────────────────────────────────────────────────┘
                                       ▲
                      ┌────────────────┘  收到 cmd_vel 时**只记速度和时刻**
                      │   （真正移动仍在控制回路上）        【第 4 条车道】
```

| 车道 | 什么时候被唤醒 | 干什么 | 归哪个回调组 |
|---|---|---|---|
| **控制回路** | 每 **20 ms** | 推进真值；判终态；回报客户 | `timer_group_`（互斥，与状态串行）|
| **状态回路** | 每 **100 ms** | 推进电量/运行时间；发状态话题 | 同上 |
| **动作回调** | 客户发目标/取消时（**时刻不定**） | 决定收不收；写意图；投信箱 | `action_group_`（**可重入**）|
| **服务回调** | 客户查询时 | **只读**，组装应答 | `service_group_`（互斥）|
| **底盘命令** | 收到 `cmd_vel` 时 | **只记下速度和时间戳** | `cmd_group_`（互斥）|

> **铁律：只有定时器改"真值"。其他人只投递"意图"。服务连意图都不投 —— 它只读。**

**为什么一把粗锁就够**（而不是 pose 锁 + 关节锁 + 导航锁 + 运动锁）：写者（两个 tick）都在 `timer_group_` 里，**写者之间根本没有并发**；唯一与写者并发的是"只读的服务"和"只写意图的动作"。所以拆锁**减少不了任何竞争**，只会新增"锁序不一致"的风险。

**为什么状态服务单独一个组**：客户查状态不该占控制回路的车道。【量】实测（6 个客户端猛灌 ~4000 req/s）：

| 场景 | 平均频率 | 抖动 std | 最坏间隔 |
|---|---|---|---|
| 无负载 | 49.999 Hz | 0.52 ms | 23 ms |
| 和控制回路**共组** | 49.896 Hz | **1.67 ms** | **51 ms** |
| **独立组**（现在） | 49.996 Hz | **0.80 ms** | **25 ms** |

吞吐几乎不受影响（−0.2%），但**抖动差一倍以上** —— 挪组挪掉的是"服务和定时器互堵"，不是"客户之间互堵"（后者在服务自己的请求队列里，与分组无关）。

### 3.1 一根"取消"的旅程（为什么它有 20 ms 延迟）

```
客户 ──取消──▶ 动作回调车道：
              handleNavCancel:
              ├─ 问框架 goal_handle->is_active()（**锁外**）
              ├─ 拿 mutex_
              ├─ slot.cancel_requested = true     ← 【投一位信箱】
              └─ 放 mutex_ → 返回 ACCEPT
              ↑ 取消回调**也不移动机器人**

┌─ 下一拍（最多 20 ms 后）────────────────────┐
│ navTick: 看到信箱 → slot.finish()           │
│          → 造 result → 锁外 handle->canceled()│
└────────────────────────────────────────────┘
```

`cancel_requested` 是**一位信箱，不是状态** —— 它**任何相位都可能到达**（连 `kIdle` 都可能：取消回调里 `is_active()` 与上锁之间目标可能已结束），所以不能并进 `Phase` 那个三态枚举。

### 3.2 三态枚举治的是什么病

原来的三个 bool（`reserved` / `active` / `cancel_requested`）**有 8 种组合，只有 4 种合法**，而 `reserved && active` 这个非法组合正是"悬空句柄 bug"的老家：目标已被接受但句柄还没到，那一瞬间 tick 落进去 → `handle->succeed()` 空指针解引用，或者动作作用在**上一个已结束的目标**上。

改成三态 + **句柄和相位在同一临界区改**之后，`kRunning` **蕴含**句柄非空 —— 由**构造**保证，不靠时序。

---

## 4. mock 里的"世界"：传感器在看什么

mock 给了一个**最简单的世界**：一个长方体房间，六个面就是全部障碍物。两个传感器**共用同一个几何函数** `rayToBox()`（标准 **slab 法**：把长方体看成三对平行板，三者求交集）。

```
        y  ▲
   ┌───────┼───────┐  y = +4
   │       │       │
   │       │       │      x ∈ [-5, 5]
───┼───────┼───────┼──▶ x  y ∈ [-4, 4]
   │       │       │      z ∈ [0, 2.5]（地面/天花板）
   └───────┼───────┘  y = -4
        x=-5     x=+5
```

| 传感器 | 怎么算出来的 | 最容易踩的坑 |
|---|---|---|
| **2D 激光** | 360 条射线（1° 一条，量程 0.05–10 m），`rayToBox` 的 `dz=0`（水平 ⇒ 地面/天花板永远命中不了，等价于"只看四面墙"）| ① 无回波用 **`inf`**（不是 0！0 会被下游当成"紧贴障碍"直接急停）② `frame_id=laser_frame`，世界角必须加 `pose_.yaw` |
| **IMU** | 朝向取真值；角速度只能由**上一拍的偏航差 / 周期**得到（mock 没有力矩/轮速模型）| 协方差给 `1e-4` **不给 0** —— ROS 里 0 表示"完全已知"，下游会当特例处理。静止时加速度计读到的是**支持力**，所以 z 是 **+9.81** |
| **深度相机** | 64×48 逐像素**针孔投影 + 射线求交**，存的是**沿光轴的 z 深度**（不是射线长度！）| 最左列偏轴 30°，打到 4.8 m 那面墙的**射线长**是 5.543，但 **z 深度仍是 4.8**。存错了两边都不报错，只是下游投影系统性偏大 |
| **CameraInfo** | 内参**从视场推出来**：`fx = (宽/2)/tan(60°/2) ≈ 55.43`，`fy = fx` | 这样内参和图像天然自洽 —— 验收里可以反推 `atan(cx/fx)` 必须正好等于半个视场 |
| **点云** | 是**深度图的另一种排法**（反投影），不是新信息 | 所以它直接吃已经算好的深度图，**不重打一遍射线**。它是有组织点云（48×64），真机的 `depth_image_proc` 也这么发 |

**QoS 必须与 SDK 侧逐字一致**：`SensorDataQoS()` + `keep_last(qos.sensor.depth)`。**不一致的后果是"静默收不到"**，不是报错。特别是 `SensorDataQoS()` 默认深度是 5，而 `qos.sensor.depth` 默认是 10 —— 不显式 `keep_last` 的话，那个配置项就是个"改了不生效"的死配置。

**`camera_link → camera_optical_frame` 那条静态 TF 必须发**：ROS 图像系是 z 前/x 右/y 下，body 系是 x 前/y 左/z 上，差一个固定旋转。少发它（或写反）**不会报错**，只会让客户所有投影结果都错 —— 而"错误但看起来合理"是这个项目最怕的一类 bug。

---

## 5. 适配器：接真栈时中间那一层

mock **直接**提供 `navigate_to_pose` / `move_to_joint_positions`，所以打 mock 不用适配器。**接真栈时必须加一层** `robot_sdk_bridge`，因为下游的动作**名字不同、形状也不一样**：

| 适配器 | SDK 这侧（server） | 翻成下游（client） | 补了什么 |
|---|---|---|---|
| `nav2_adapter` | `sdk/navigate_to_pose` | `nav2_msgs/NavigateToPose` 的 `navigate_to_pose` | 基本是转发（形状几乎一样）。`max_speed`/`avoid_obstacles` 兑现不了 → 打 WARN |
| `motion_adapter` | `sdk/move_to_joint_positions` | `/joint_trajectory_controller/follow_joint_trajectory` | 轨迹起点取自 `/joint_states`；把"一个点"补成两点轨迹；算时长；自己算完成百分比 |
| `moveit_adapter` | `sdk/move_to_pose` | `/move_action`（`moveit_msgs/MoveGroup`）| 位姿拆成**约束**；`max_velocity` → `max_cartesian_speed`（m/s，不是比例）；末距**执行完从 TF 实测**，不拿规划终点冒充 |

**为什么必须换名（`sdk/` 前缀）**：action 底层是 **service**，而 service 名是**全局唯一**的。Nav2 已经占了 `navigate_to_pose`，同名但**类型不同**的两个 action server 无法共存。所以 SDK 侧必须换名，客户只改**配置里一个动作名**。

**这一层买到的是解耦**：客户只依赖 `robot_sdk_interfaces`。换后端（mock → Nav2 → 真机）**只改配置里的一个名字**。

---

## 6. 换成 Gazebo：谁从 mock 手里接管了什么

```bash
ros2 launch robot_sdk_gazebo gazebo.launch.py                  # 带 GUI
ros2 launch robot_sdk_gazebo gazebo.launch.py headless:=true    # 无显示器 / 省内存
```

启动顺序（`gazebo.launch.py`）：gz 仿真 → `robot_state_publisher` → **桥** → `odom_tf_broadcaster` → **4 秒后**生成机器人（`ros_gz_sim create -topic robot_description`）→ **12 秒后**起两个 `spawner`。那两个延迟是因为 `gz_ros2_control` 插件要等机器人被生成出来才建 `controller_manager`。

### 6.1 每个部件由谁驱动

| 部件 | mock 里是谁 | Gazebo 里换成了谁 | 过桥吗 |
|---|---|---|---|
| 手臂 7 关节 | `motionTick` **假插值**（命令即状态） | **`gz_ros2_control`**（真物理）+ `arm_controller` | ❌ 不过桥（启动用 `spawner`；`/joint_states` 由 `joint_state_broadcaster` 直接在 gz 进程内发进 ROS）|
| 底盘（差速） | `cmdVelTick` **理想积分**（无加速度/无打滑） | gz 的 **`DiffDrive`** 插件（**没进 ros2_control**）| ✅ `/cmd_vel` **ROS→GZ**、`/odom` **GZ→ROS** |
| 车轮关节状态 | （mock 不发车轮，只发臂的 7 个） | gz 的 `JointStatePublisher`（**只列车轮**）| ✅ `/joint_states` GZ→ROS |
| 激光 | `makeScan` 射线求交 | gz `gpu_lidar` 传感器（**渲染式**，需要 EGL/GPU）| ✅ `/scan` GZ→ROS |
| IMU | `makeImu` | gz `Imu` 系统 | ✅ `/imu` GZ→ROS |
| 深度相机 | `makeDepthImage` 针孔投影 | gz `depth_camera` 传感器 | ✅ 三条 GZ→ROS（图/内参/点云）|

**桥一共 9 条，只有 `/cmd_vel` 一条是 ROS→GZ**，其余 8 条都是 GZ→ROS（`ros_gz_bridge.yaml`）。手臂命令**不在桥里** —— 它归 ros2_control。

### 6.2 `odom_tf_broadcaster` 为什么存在

gz 的 `DiffDrive` **只发 `/odom` 话题，不发 TF**；而 Nav2 要的是 `map → odom → base` 这条**完整的 TF 链**。所以专门有个小节点把 `/odom` 抄成 `odom → base_footprint` 的 TF。

它订阅 `/odom` 用 **`SensorDataQoS`** —— 和发布端必须一致，不一致会**静默收不到**（同一个坑，第三次出现）。

### 6.3 ⚠️ 一个真实的缺口：`map → odom` 没人发

| 环境 | 谁发 `map → odom` |
|---|---|
| **mock** | **mock 自己**。它是**单位变换**（假装 odom 与 map 重合）—— 等于"定位永远完美" |
| **Gazebo** | **没有**。仓库里没有任何 SLAM / AMCL 配置，`gazebo.launch.py` 也不起 Nav2 |
| **真机** | **没有**。同上（只有注释提到"由 SLAM 或 AMCL 给"）|

后果：**Nav2 那条链路在 Gazebo 里跑不起来（缺一环定位）**。想跑就得自己补一个 AMCL 或 SLAM，并让 Nav2 用 `use_sim_time:=true`（否则仿真时钟和墙钟差 1.7e9 秒）。

> 这不是 bug，是**还没做**。写在这里是为了"别装作没有"。

### 6.4 Gazebo 特有的坑（简述）

| 坑 | 说明 |
|---|---|
| `hardware:=gazebo` | xacro 展开时把硬件插件从 `mock_components/GenericSystem` 换成 `gz_ros2_control/GazeboSimSystem`，并删掉那 7 个 gz `JointPositionController`（否则会和 ros2_control **抢同一关节**）|
| `ros2_control_params` **必须绝对路径** | 默认留空、由 launch 传。写 `$(find ...)` 会在构建这个包**自己**时抛 `PackageNotFoundError` |
| `spawner` 必须用 `ExecuteProcess` | `launch_ros` 的 `Node` 会往命令行补一个光秃秃的 `--ros-args`，spawner 当成"params-file 缺路径"直接退出 ⇒ 一个控制器都加载不了 |
| `use_sim_time` | 必须给**规划侧**也加 `use_sim_time:=true`，否则差 1.7e9 秒 |
| 内存 | 【量】headless 全套 ~1.1 GB、约 3 分钟，起栈后只剩 ~1 GB，再叠东西会被 OOM 杀掉（表现为"整个栈莫名其妙消失"）|

---

## 7. 换成真机：什么会变、什么**不**变

### 7.1 不变的（这才是这套设计的意义）

| 不变的 | 为什么 |
|---|---|
| **客户代码一行不改** | 客户只依赖 `robot_sdk_interfaces` 和 `robot_sdk` |
| 话题/服务/action 的**名字** | 全是相对名，由 rcl 按命名空间展开 |
| SDK 的 API 和错误语义 | `Result` / `Error` / `RobotSdkError` 与硬件无关 |
| 契约里的帧名、容差、超时 | 都写在 `RobotConfig` 里 |

### 7.2 变的：执行侧换人

| | mock（`colcon test` 走这条） | Gazebo | 真机 |
|---|---|---|---|
| 关节驱动 | `motionTick` 假插值 | `gz_ros2_control` 真物理 | **ros2_control 的硬件接口**（`SystemInterface`）→ 真实电机/编码器/总线（CAN/EtherCAT…）|
| 硬件插件 | `mock_components/GenericSystem`（**命令即状态**，`open_loop_control: true`）| `gz_ros2_control/GazeboSimSystem` | 厂家/自研的硬件接口（`open_loop_control: false`）|
| 底盘 | `cmdVelTick` 理想积分 | gz `DiffDrive` | 真实差速驱动器 |
| 传感器 | 解析生成 | gz 渲染式传感器 | 真实驱动（噪音、丢帧、延迟、畸变都会有）|
| **限位/能不能到** | 从 URDF 读，当场拒 | 由控制器/物理决定 | **由控制器/驱动器决定** ← 见下 |
| 谁发 `map→odom` | mock 自己（单位变换）| 缺 | **缺，必须补**（SLAM/AMCL）|

**"限位归谁判"这条纪律**：关节的类型、限位、速度上限是**机器人的属性**（写在它自己的 URDF 里），所以"这个目标能不能到"**归执行侧判**，SDK 不替它做（SDK 是硬件无关的，两处各存一份规则迟早漂移）。

- mock ✅ 会拒（前提是 fixture 把 URDF 喂给它）
- 真机上将来是**控制器/驱动器**拒 ✅
- SDK ❌ 不判

**换真机要多什么**（清单）：

1. 把 `<ros2_control><hardware><plugin>` 换成真机的硬件接口
2. **补定位**：得有人发 `map → odom`（SLAM 或 AMCL），否则 Nav2 的全局规划没有依据
3. 接受传感器的**不完美**（噪声/丢帧/延迟）—— mock 的"解析解"验收那套判据在真机上要改成统计判据

---

## 8. 三条路并排总表

| | **mock** | **Gazebo** | **真机** |
|---|---|---|---|
| 它是什么 | 一个 ROS 节点，假装自己是机器人 | 物理仿真 | 真的机器人 |
| 谁当执行侧 | `mock_robot_node` 一个进程 | gz + `gz_ros2_control` + 桥 | 控制器 / 驱动器 + 硬件接口 |
| 传感器来源 | 解析生成（射线求交） | gz 渲染式传感器 | 真实驱动 |
| 手臂驱动 | 假插值（命令即状态） | 真物理（`gz_ros2_control`） | 真电机 |
| 底盘驱动 | `cmdVelTick` 理想积分 | gz `DiffDrive` | 真差速驱动器 |
| 谁发 `odom→base` | mock（真值位姿） | `odom_tf_broadcaster`（抄 `/odom`） | 轮式里程计 |
| **谁发 `map→odom`** | **mock（单位变换 = 定位永远完美）** | **缺** | **缺，必须补** |
| `/odom` 话题 | **没有**（只发 TF） | 有（gz 发、桥转发） | 有（驱动器发） |
| 有没有噪声 | 没有（理想值） | 有（仿真级） | 有（真实级） |
| 定位准不准 | 永远完美 | 准 | 会漂 |
| 接 Nav2 | ✅ 能跑（`robot_sdk_nav2` 那条，本文 §9 有实测）| ❌ 缺 `map→odom` | ❌ 缺 `map→odom` |
| 接 MoveIt | ✅（move_group 直接读 URDF）| ✅ | ✅（真控制器） |
| 内存/代价 | 小 | 大（~1.1 GB）| — |
| 适合干什么 | 日常开发、集成测试、CI | 验"物理上跑不跑得动" | 真实验收 |

---

## 9. 实测记录（本节的数据都是【量】）

### 9.1 发一条 `cmd_vel`，到底什么变了

起 mock，发 `linear.x = 0.3 m/s`（20 Hz，2 秒），前后各采一次：

| | 发之前 | 发之后 | 变化 |
|---|---|---|---|
| TF `map → base_footprint` | `[0.000, 0.000, 0.000]` | `[0.462, 0.000, 0.000]` | **前移 0.462 m** |
| `/scan` 前（0°） | 4.90 m | 4.44 m | **−0.46** |
| `/scan` 后（180°） | 5.10 m | 5.56 m | **+0.46** |
| `/scan` 左（90°）/ 右（−90°） | 4.00 / 4.00 | 4.00 / 4.00 | **不变**（只往前走了）|
| `/joint_states`（关节） | 全 0 | 全 0 | **原封不动** |

**这张表说明三件事**：① `cmd_vel` 只改位姿；② 但激光是**真的从新位姿重新求交**的（所以传感器数据会跟着变，不是只挪了个坐标标签）；③ 手臂关节完全不动（车轮也不转 —— mock 不发车轮关节）。

（前移量比"0.3 m/s × 2 s = 0.6 m"小一点，是因为 `ros2 topic pub` 启动要花时间，实际有效时长约 1.5 s；另外停止后看门狗还有 0.5 s 尾巴。）

### 9.2 什么算"只更新位置"

它更新的是 **`pose_`：(x, y, yaw)** —— 是**位姿**（位置+朝向），不只是位置。而且 `pose_` 是 mock 的**唯一真相源**，改它等于改了整个世界对这台机器人的看法：TF、激光、点云、导航反馈、SDK 的 `currentPose()` 全都跟着变。

它**不**更新：关节（车轮不转，`/joint_states` 里 velocity 全 0）、速度状态、也没发 `/odom`。

### 9.3 Nav2 那条链路（"规划"和 "cmd_vel" 是两回事）

**先说结论**：Nav2 **规划出来的东西不是 `cmd_vel`**。

```
planner_server（NavfnPlanner）──▶ /plan     （nav_msgs/Path，一条路）
                                       │
                                       ▼
controller_server（RotationShim + RPP）──▶ /cmd_vel  （geometry_msgs/Twist，20 Hz 连续发）
                                       │
                                       ▼
                                   mock 的 cmd_vel 订阅 → cmdVelTick → 改 pose_
```

- `/plan` 是**"规划好的结果"**（一条路径），由 `planner_server` 发。
- `/cmd_vel` 是**"这一刻该怎么动"**的**连续指令流**，由 `controller_server`（跟踪路径）和 `behavior_server`（spin / backup / wait 这些恢复动作）发。**不是发一次**。

实测（`ros2 launch robot_sdk_nav2 navigation.launch.py`，起 mock + Nav2 最小集 + 适配器）：

| 观察 | 实测值 |
|---|---|
| `/plan` 的发布者 | `planner_server`（唯一）|
| `/plan` 消息 | `nav_msgs/Path`，**237 个路径点**，`frame_id = map` |
| `/cmd_vel` 的发布者 | **5 个**：`controller_server` ×1 + `behavior_server` ×4（恢复动作各一个）|
| `/cmd_vel` 的订阅者 | `mock_robot` ×1 |
| `/cmd_vel` 实测速率 | **19.9 Hz**（配置是 `controller_frequency: 20.0`）|
| `linear.x` 范围 | 0.000 ~ **0.400** m/s（配置 `desired_linear_vel: 0.4`）|
| `angular.z` 范围 | −1.000 ~ **+1.000** rad/s（配置 `rotate_to_heading_angular_vel: 1.0`）|
| 一次导航目标 | `SUCCEEDED`，`distance_remaining: 0.1414`，终点实测 `(1.882, 0.916)`（目标 `(2.0, 1.0)`）|

也就是说：**Nav2 真的把 mock 机器人开到了目标点**，靠的就是 `/cmd_vel` 这一条路 —— mock 这边只有 `cmdVelTick` 会因它而改变位姿。

> 顺带一个观察【推】：`nav2_params.yaml` 里 `bt_navigator.odom_topic: /odom`、以及 RPP 默认的 `odom_topic`，都指向一个 **mock 没有的话题**（mock 只发 `odom→base` 的 **TF**）。实测导航照样成功 —— 所以这几个参数在这条路上没起作用（Nav2 要的位姿是从 **TF** 拿的）。换成 Gazebo/真机时 `/odom` 就存在了，那时它们才有意义。

---

## 10. 常见误解

| 误解 | 实际 |
|---|---|
| "忘了 `spin()` 只是慢一点" | ❌ **完全收不到**。目标回应、进度、结果全都要靠执行器跑回调 |
| "Nav2 规划好了，就直接发 `cmd_vel`" | ❌ 规划是 `/plan`（Path），`cmd_vel` 是**控制器**每 20 ms 发一次的连续速度指令 |
| "`cmd_vel` 让机器人动，所以传感器也跟着变" | ✅ 对，但原因不是"cmd_vel 改传感器"，而是传感器每次都是从**新位姿**重新求交算出来的 |
| "状态既有话题又有服务，是冗余" | ❌ 回答的是不同问题：**推送的最新值** vs **此刻的确切值**。冗余是指同一份数据存两处（那个已经消灭了）|
| "QoS 不一致会报错" | ❌ **静默收不到**（不报错、只是没数据）。这个坑在本项目出现了三次 |
| "取消应该立刻停" | 最多一个控制周期（20 ms）的延迟 —— 因为取消只投一位信箱，真正的"停 + 回 canceled"由控制回路做 |
| "Gazebo 是仿真，所以 Nav2 直接能用" | ❌ Gazebo 那条**缺 `map→odom`**（没人做定位）。mock 能跑是因为 mock 自己发了一条单位变换 |
| "mock 和 Gazebo 差不多" | mock 是**一个 ROS 节点假装机器人**（理想值）；Gazebo 是**真物理**（有噪声、有惯性、有渲染开销）|
| "SDK 会替我检查关节限位" | ❌ 限位是**机器人的属性**，归**执行侧**判（mock ✅ / 真机上的控制器 ✅ / SDK ❌）|

---

## 11. 想再往下挖，看哪里

| 想了解 | 看 |
|---|---|
| mock 内部设计（为什么三态、为什么一把锁、凭证式加锁） | `mock-robot-node.md` |
| SDK 调用流程、`spin()`、与手写 ROS 2 的对比 | `sdk-call-flow.md` |
| 怎么看机器人在哪 / 3D 空间 / URDF 怎么发布 | `visualization.md` |
| Gazebo 那条链路的起法与已知坑 | `src/robot_sdk_gazebo/README.md` |
| 验收探针怎么跑、为什么老挂住 | `src/robot_sdk_tests/README.md` |
| Python 绑定的边界约定 | `src/robot_sdk_py/README.md` |
| 逐次踩坑的历史 | `学习笔记.md` |
