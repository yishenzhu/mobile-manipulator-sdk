# SDK 调用流程 —— 与「手写 ROS 2」的对比

> **面向**：不熟 ROS 2 的人。
> **本文的代码只表示"流程形状"和"接口长什么样"，不是可运行的完整程序。**
> 能跑的成品在 `ws/src/robot_sdk_examples/`。
> **姊妹文档**：`mock-robot-node.md`（讲 mock **内部**怎么设计）。

---

## 0. 先建立三个直觉

后面所有内容都从这三句长出来：

| # | 直觉 | 一句话 |
|---|---|---|
| ① | **机器人程序大半时间在"等"** | 等消息、等结果、等超时。真正"算"的时间很短 |
| ② | **等到的东西 → 走"回调"** | 消息到了不是"你去拿"，而是**系统调你写的小函数** |
| ③ | **必须有个东西一直在"值班"** | 没人值班，等的东西永远到不了你手上 |

**第 ③ 句就是 `robot.spin()`。** 它是本文反复出现的那根线。

---

## 1. 传统写法：手写 ROS 2，你要自己干哪些活

需求很简单：**读一次机器人状态，然后让它去 (1.5, 0.0)**。

### 1.1 你必须自己做完这 9 件事

```text
①  建一个"节点"                              —— 参与通信的基本单元
②  建状态订阅，并写一个回调把状态**存起来**     —— "最新值"得自己管
③  想清楚订阅的通信质量参数（队列多深等）        —— 设错了会静默收不到
④  建一个"导航动作客户端"，并指明动作名和消息类型
⑤  自己拼目标消息                              —— 连时间戳、朝向都得自己填
⑥  自己写三个回调：服务端回应 / 中途进度 / 最终结果
⑦  自己处理"异步结果"                          —— 包括怎么等、等到没有
⑧  自己区分"流程成功"和"机器人真到位了"           —— 这是两回事
⑨  自己解决一对矛盾：**"等结果" 和 "让消息流转" 不能同时干**
```

### 1.2 这 9 件事，分别要碰哪个接口

```cpp
// ① 节点：自己继承
class MyApp : public rclcpp::Node { ... };

// ②③ 订阅 + 通信质量 + 回调（"最新值"自己存）
status_sub_ = create_subscription<RobotStatus>(
    "robot_status", /*qos=*/10,
    [this](RobotStatus::SharedPtr msg) { last_status_ = msg; });

// ④ 动作客户端：自己指明动作名和消息类型
nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

// ⑤ 自己拼目标消息（连时间戳、朝向都得自己填）
goal.target_pose.header.frame_id = "map";
goal.target_pose.header.stamp = now();
goal.target_pose.pose.orientation.w = 1.0;

// ⑥ 自己写三个回调
options.feedback_callback = ...;   // 中途进度
options.result_callback   = ...;   // 最终结果
//    （还有"服务端回应"那一个）

// ⑦ 异步结果：自己看 future 就绪没有
auto goal_future   = nav_client_->async_send_goal(goal, options);
auto result_future = nav_client_->async_get_result(goal_handle);

// ⑧ 两层成功：终态码 和 业务字段 分别判
wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result->success

// ⑨ 等 + 转消息：自己想办法同时干
rclcpp::spin_until_future_complete(node, future, timeout);
```

> **第 ⑨ 条是传统写法里最绕的一块。** 单线程里你要么在等、要么在转消息 ——
> 想两者兼得，只能用一个"边等边转"的特殊等待方式，或者自己开线程。
> **这**正是 SDK 替你包掉的那件事。

### 1.3 传统写法的数据流

```text
      你的程序（手写）                           mock 机器人
 ┌───────────────────────────┐        ┌───────────────────────────┐
 │  你自己建的节点             │        │  模拟机器人节点             │
 │   ├─ 订阅状态  ────────────┼─ 话题 ─▶│   ├─ 状态发布              │
 │   ├─ 订阅激光 / IMU / 相机  │        │   ├─ 传感器发布            │
 │   ├─ 监听坐标变换           │        │   ├─ 坐标变换广播          │
 │   └─ 导航动作客户端 ────────┼─ 动作 ─▶│   └─ 导航动作服务端        │
 │                            │◀ 进度 ──┤                          │
 │                            │◀ 结果 ──┤                          │
 │                            │        │                          │
 │  这些**全是你的活**：        │        │                          │
 │   存最新值 · 拼消息          │        │                          │
 │   处理异步结果 · 让消息流转   │        │                          │
 └───────────────────────────┘        └───────────────────────────┘
```

---

## 2. SDK 写法：你只需要做 3 件事

### 2.1 三步

```cpp
// ① 告诉 SDK "我要连哪台机器人"
robot_sdk::RobotConfig config{"my_app"};   // 名字、超时…都有默认值
robot_sdk::Robot robot(config);

// ② 开一个后台线程让它 spin（让通信管道通水）
std::thread spinner([&robot]() { robot.spin(); });

// ③ 写业务
//    注意：waitForXxx 返回的是**那一条**消息，字段就从它上面读；
//    别一边拿着这一条、一边又去调 API 读"此刻最新那一条"（两个来源，未必相同）。
if (auto status = robot.status().waitForStatus(5s); status.ok()) {
  std::printf("电量 %.1f%%\n", status.value()->battery_percentage);
}

robot_sdk::NavigationGoal goal;
goal.target = {1.5, 0.0, 0.0};      // x, y, 朝向 —— 就这三样
if (auto r = robot.navigation().goTo(goal); r.ok()) {
  r.value().success;                 // 真的到了吗（见 §6）
}

robot.stop();
spinner.join();
```

**对照 §1.2 看**：`create_subscription` / `create_client` / `async_send_goal` / `async_get_result` /
`spin_until_future_complete` / `header.stamp` / `orientation.w` —— **一个都不用出现。**

### 2.2 两条会反复用到的约定

#### ① 结果要自包含：一个代码块里只用**一个来源**

同一个东西，SDK 里往往**既**能"从拿到的那一份里读"**也**能"从 API 上问最新的"。
混用不会报错，但会让"读到的是同一个值"这件事**失去保证**，也会误导读代码的人。

```cpp
// 好：这一条自己就是完整的 —— 字段都从它读
if (auto status = robot.status().waitForStatus(5s); status.ok()) {
  status.value()->battery_percentage;
  status.value()->is_healthy;
  status.value()->is_charging;
}

// 差：手上拿着"这一条"，又去 API 读"此刻最新那一条" —— 两个来源
if (auto status = robot.status().waitForStatus(5s); status.ok()) {
  status.value()->state;
  robot.status().batteryPercentage();   // ← 这读的是**最新的**，未必是上面那条
}
```

**设计上也朝这个方向走** —— 能放进"返回的那一份"里的，就不让客户再去别处查：

- `queryStatus()` 返回**按值**的一整份，不是缓存快照的指针
- **判断类的东西不塞进消息**：`RobotStatus` 只有子字段，没有 `is_healthy` 这种合成字段 ——
  客户一行就能自己算；写进消息会把"什么算健康"冻进契约，改判据就得让所有客户重编
- `isHealthy()` 只是"**缓存里最新那条**健康吗"的便捷方法；要判断**某一份特定的**状态，
  请从那一份的子字段自己算（来源才一致）

#### ② 消息字段不回答"现在"

`battery_percentage`、`state` 这些字段，描述的都是**生成这条消息的那一刻**。
想知道"**现在**"，靠的是另外几件事：

| 你想知道 | 该看什么 |
|---|---|
| **这一条**说什么 | 字段本身（`battery_percentage` / `state` / `error_code` …） |
| **这条还新不新**（还能不能信） | 消息的 **`header.stamp`** —— 和当前时间比 |
| **此刻到底怎样**（要新鲜的） | **用服务查询** —— 请求/应答，返回的是**刚刚算出来**的一份 |
| **压根还没收到过** | 那是**客户端**的状态，消息里表达不了（客户端的等待接口会回答它） |

> **话题是"流"，可能已经旧了；服务是"问一次"，那才是新鲜的。**
> 这也是为什么 SDK 同时提供了"等一条状态"和"查一次状态"两条路。

### 2.3 对比表（**诚实版**）

| | 手写 ROS 2 | SDK |
|---|---|---|
| 行数 | 差不多 | 差不多 |
| **必须先懂的概念** | 9 个（见 §1.1） | **2 个**（机器人对象、结果对象） |
| 管"最新值" | 自己写 | 现成的等待接口 |
| 拼目标消息（时间戳/朝向） | 自己填 | 只需给"x、y、朝向" |
| 等结果 + 让消息流转 | 自己想（§1.1 第 ⑨ 条） | 后台 spin 已经解决 |
| **换后端**（mock → 真 Nav2 → 真机） | **改代码** | **只改配置里的一个名字** |
| 错误处理 | 各种异常/结果码 | 统一的"结果对象" |

> **SDK 省的不是行数，是"你必须先懂的东西"，以及"换后端时要改的地方"。**

---

## 3. 一次导航：端到端时序（本文核心）

### 3.1 时序

```text
 主线程             spin 线程         客户端的线程池        mock 的线程池
   │                  │                   │                   │
   │ ① 建机器人对象 ────┴─ 建好节点/订阅/客户端（**这一步不通信**）
   │                  │                   │                   │
   │ ② 启动 spin ────▶ 停在 spin 里等       │                   │
   │                  │                   │                   │
   │ ③ 发目标          │                   │                   │
   │   （立即发出） ────┼───────────────────┼── 目标送到 ───────▶ 接受 / 拒绝
   │                  │                   │                   │
   │                  │◀─── 回应到达 ──────┤                   │
   │                  │  工人取活 → 调回调  │                   │
   │                  │                   │                   │
   │ ④ 等结果          │                   │                   │ 每 20 毫秒：
   │   （阻塞在这里）   │                   │                   │   推进位姿一步
   │                  │                   │                   │   组装一条进度
   │                  │◀───── 进度到达 ────┤                   │ 发出进度
   │                  │  回调把它交给你的函数 │                   │
   │                  │                   │                   │
   │                  │◀───── 结果到达 ────┤                   │ 判定终态
   │ ⑤ 被唤醒 ◀────────┘                   │                   │
   │   （拿到：成功吗 / 末距多少 / 说明）     │                   │
   │                  │                   │                   │
   │ ⑥ 叫停 ─────────▶ spin 返回 ──────────┘                   │
```

### 3.2 每一步的接口

```cpp
// ① 建机器人对象 —— 只建管道，不通信
robot_sdk::RobotConfig config{"my_app"};
config.navigation.action_name = "navigate_to_pose";   // mock 用的名字
robot_sdk::Robot robot(config);

// ② 启动 spin —— 它是"泵"，不是"管道"
std::thread spinner([&robot]() { robot.spin(); });

// ③ 发目标 —— **不需要 spin**，立即返回一个句柄
robot_sdk::NavigationGoal goal;
goal.target      = {2.0, 1.0, 0.0};
goal.max_speed   = 0.5;    // <= 0 表示"由后端自己决定"
goal.timeout_s   = 20.0;   // <= 0 表示"用配置里的默认值"
std::shared_ptr<ActionHandle<Action>> handle = robot.navigation().goToAsync(goal, 你的反馈回调);
// （`Action` 写作 `NavigationAPI::Action`；实际写 `auto handle = ...` 更省事）

// ④ 等结果 —— **要 spin**；内部是"等它被回调填好"
Result<ActionHandle<Action>::WrappedResult> wrapped = handle->wait(25s);

// ⑤ 两层语义，分开判（见 §6）
wrapped.ok();                    // 流程走完了吗
wrapped.value().code;            // 终态码：SUCCEEDED / CANCELED / ABORTED
wrapped.value().result->success; // 机器人真的到了吗
```

**还有一个更省事的写法** —— "发 + 等"合成一步：

```cpp
Result<NavigationResult> r = robot.navigation().goTo(goal);
r.ok();             // 流程
r.value().success;  // 业务
```

### 3.3 「发」和「收」的分界（重要）

| 操作 | 对应接口 | 要不要 spin |
|---|---|---|
| **发**目标 | `goToAsync()` / `goTo()` 的发送部分 | ❌ 不要 |
| 收**目标回应**（接受/拒绝） | `handle->waitForAcceptance(t)` | ✅ 要 |
| 收**中途进度** | 你注册的反馈回调 | ✅ 要 |
| 收**最终结果** | `handle->wait(t)` | ✅ 要 |
| 收**服务应答** | `status().queryStatus(t)` | ✅ 要 |
| 收**话题** | `status().waitForStatus(t)`、`perception().waitForLaserScan(t)` | ✅ 要 |

> **一句话：spin 管"进门"的一切；"出门"不等它。**

### 3.4 接上适配器后，流程哪里变了

```text
     客户代码             ← 不变
        │
        ▼
   导航动作（动作名字：sdk/navigate_to_pose）
        │
   ┌────┴─────┐
   │  适配层    │   ← **只有这一层换了**
   └────┬─────┘
        │
        ▼
     真 Nav2 ──▶ 底盘
```

**客户代码里改的只有一处**：

```cpp
config.navigation.action_name = "sdk/navigate_to_pose";   // 就这一行
```

这就是"接口解耦"的具体含义：**换掉导航栈，客户代码不动。**

> 这层适配器现在就在 `ws/src/robot_sdk_bridge/`（`nav2_adapter`）。
> 它跑起来要和下游同时起：
> ```bash
> ros2 run robot_sdk_bridge nav2_adapter      # 起适配器
> ros2 run robot_sdk_examples demo_navigation sdk/navigate_to_pose   # 客户打适配器
> ```

**适配器上的"后端特有旋钮"** —— SDK 契约里**没有**这些东西（它们只对某个后端有意义），
所以只能在这一层调：

| 旋钮 | 是什么 | 运行时能改吗 |
|---|---|---|
| `behavior_tree` | Nav2 用哪棵"出事怎么办"的流程图（XML 路径，空 = 用默认） | ✅ **每次发目标时读** —— 改了对**下一个**目标生效 |
| `default_frame` | 目标没写 frame 时按哪个帧处理 | ❌ 启动时定 |
| `sdk_action_name` / `nav2_action_name` | 两侧的动作名 | ❌ 启动时定（server/client 已经建好了）|

> **判据**：一个参数如果只在启动时读，就必须**说清它是启动时定的** ——
> 否则就成了"改了不生效"的死配置。`behavior_tree` 原本是这种（只在构造时读一次），
> 现在改成每次发目标时读。（反向的例子：SDK 的 `max_acceleration` **声明了但没有任何后端兑现**。）

---

## 4. `spin()` / 执行器 / 线程池（最容易搞错的部分）

### 4.1 `spin()` 在干什么

**不是"把回调扔给线程池"，而是"让工人上班、然后等他们下班"。**

```text
线程池里每个工人的循环：

   ① 问中间件："有活吗？"          （有消息？定时器到点？有请求？）
   ② 有 → 把这份活**连参数一起取走**
   ③ 执行 → 也就是**调你写的回调**
   ④ 回到 ①
```

`spin()` 的调用线程本身**不跑回调**，它只是启动这些工人并等它们。

```cpp
robot.spin();                          // 一直值班（阻塞，直到 stop()）
robot.spinSome(std::chrono::milliseconds(100));  // 值 100 毫秒的班就下班
```

> **回调不是被"派"下去的，是工人自己"抢"来的。** 这个区别决定了 §4.3。

### 4.2 线程池有多少线程

执行器的官方文档里写得很明确：**线程池的线程数默认取 CPU 核数，最少 2 个。**
（出处：`multi_threaded_executor.hpp` 第 49-50 行的参数说明。）

SDK 用的就是默认值，所以在 8 核机器上跑一个示例，大约是：

```text
主线程（写业务） + spin 线程（在等） + 8 个工作线程（回调跑在这里）
```

### 4.3 回调组是**给谁**看的

既然回调是"抢"来的，那"能不能同时抢"就得有个规矩 —— 这就是**回调组**。

执行器的文档原意是：**如果某个组里已经有一个实体正在被执行，而组策略不允许第二次获取（比如互斥），那么这个"可获取"标志就会变成否。**
（出处：`callback_group.hpp` 第 190 行。）

工人抢活之前会先瞄一眼这个标志：

| 组类型 | 效果 |
|---|---|
| **互斥** | 有人在跑这个组的活 → **别人抢不了** → 一次一个 |
| **可重入** | **允许**第二个工人同时拿 → 真的并发 |

**SDK 里的三个组**：

| 组 | 类型 | 为什么 |
|---|---|---|
| 控制 | 互斥 | 同一时刻只处理一个目标，避免两个导航目标打架 |
| 感知 | **可重入** | 多路传感器彼此独立又高频 —— 串行会成瓶颈 |
| 状态 | 互斥 | 目的是**隔离**，不让状态回调挤占控制路径 |

> ⚠️ **同一个互斥组只保证"不并发"，不保证"同一个线程"** ——
> 这批活在工人 1，下批可能在工人 2。

### 4.4 线程归属速查

| 谁 | 在哪条线程上 |
|---|---|
| 你写的业务逻辑 | **主线程** |
| spin 本身 | **spin 线程**（它在等） |
| SDK 的所有回调（状态/传感器/进度/结果） | **客户端的线程池** |
| mock 的定时器回调（控制回路、状态回路） | **mock 的线程池**（互斥组） |
| mock 的动作回调（收目标、取消） | **mock 的线程池**（可重入组） |

**两边各有一套** —— 这就是为什么 mock 里那些锁和回调组不是过度设计。

---

## 5. 主线程在干什么：**只做决策，不搬消息**

| 谁 | 职责 | 类比 |
|---|---|---|
| **主线程** | 决定去哪、什么时候去、结果怎么办 | 老板 |
| **spin + 线程池** | 收发消息、把消息变成函数调用 | 收发室值班员 |
| **机器人（它自己的线程池）** | 真的移动、算传感器 | 车间工人 |

如果业务是长跑的（比如巡逻），主线程就是一个很朴素的循环：

```text
   ┌─────────────────────────────┐
   │  去 A 点                     │
   │  等它到                      │◀──┐
   │  去 B 点                     │
   │  等它到                      │───┘ 一直转
   └─────────────────────────────┘
```

spin 在后台一直值班 —— 主线程从头到尾**没碰过"搬消息"**这件事。

---

## 6. 两层"成功"（很容易混）

```text
              ┌─────────────────────────────┐
第一层         │  流程走完了吗？               │  ← 通信层的事
              │  服务端不在 / 超时 → 没走完    │
              └─────────────┬───────────────┘
                            │ 走完了
              ┌─────────────▼───────────────┐
第二层         │  机器人**真的到位**了吗？      │  ← 业务层的事
              │  被取消 / 被拒绝 → 没到位     │
              └─────────────────────────────┘
```

**两条路径上，这两层的接口分别是**：

```cpp
// 路径 A：阻塞式（goTo）
Result<NavigationResult> r = robot.navigation().goTo(goal);
r.ok();              // 第一层：流程
r.value().success;   // 第二层：业务
r.error().toString();// 第一层失败时的原因（可读文字）

// 路径 B：非阻塞（goToAsync + wait）
auto wrapped = handle->wait(25s);
wrapped.ok();                     // 第一层：流程
wrapped.value().code;             // 第一层：终态码（SUCCEEDED/CANCELED/ABORTED）
wrapped.value().result->success;  // 第二层：业务
```

| | 第一层 | 第二层 |
|---|---|---|
| 回答的问题 | 流程走完了吗 | 机器人真的到位了吗 |
| 为"否"的例子 | 服务端不在、超时、被拒绝 | 被取消、中途失败 |

**别把它们混为一谈** —— 这是 SDK 里最值得记的一条约定。

---

## 7. 流程相关接口速查

> **下面所有 `Duration` 都指 SDK 里的 `std::chrono::duration<double>`**（秒）。
> 只贴**流程相关**的接口，不追求穷尽。

### 7.1 入口：`Robot`

```cpp
Robot(const RobotConfig& config);                        // 自建节点 + 执行器
Robot(const Node::SharedPtr& node, const RobotConfig& config);  // 寄生到你自己的节点上

NavigationAPI&   navigation();
MotionAPI&       motion();
PerceptionAPI&   perception();
StatusAPI&       status();
TransformManager& tf();                                  // 坐标变换

void spin();                                             // 阻塞值班
void spinSome(std::chrono::milliseconds timeout);        // 值一会儿班
void stop();                                             // 让 spin 返回
bool ownsExecutor() const;                               // 执行器是不是我建的？
```

### 7.2 导航

```cpp
struct NavigationGoal {
  Pose2D target{};                        // x, y, 朝向
  std::string frame{};                    // 空 = 用配置里的全局帧
  double max_speed{0.0};                  // <= 0 = 由后端决定
  bool avoid_obstacles{true};             // ⚠️ 当前没有后端兑现它
  double timeout_s{0.0};                  // <= 0 = 用配置里的默认值
};

struct NavigationFeedback {
  Pose2D current_pose{};
  double distance_remaining{0.0};
  double estimated_time_remaining{0.0};
  std::string state{};
};

struct NavigationResult {
  bool success{false};                    // 第二层：真的到了吗
  double final_distance_error{0.0};
  std::string message{};
};

bool serverAvailable(Duration timeout);
std::shared_ptr<ActionHandle<Action>> goToAsync(const NavigationGoal& goal,
                                                FeedbackCallback on_feedback = nullptr);
Result<NavigationResult> goTo(const NavigationGoal& goal);
Result<Pose2D> currentPose(Duration timeout = Duration{0.1});
void cancelAll();
```

### 7.3 那个"目标句柄"：`ActionHandle`

```cpp
bool valid() const;
void onFeedback(FeedbackCallback callback);                        // 挂进度回调

// 这两个是模板 —— 所以传 25s / 200ms 这类字面量都行
template <class Rep, class Period>
bool waitForAcceptance(const std::chrono::duration<Rep, Period>& timeout);  // 服务端收不收

template <class Rep, class Period>
Result<WrappedResult> wait(const std::chrono::duration<Rep, Period>& timeout);  // 等最终结果

void cancel();                                                     // 请求取消（任意线程可调）
```

> **`wait()` 超时不是撒手不管** —— 它会**主动取消**。
> 否则服务器上的目标还在跑、机器人还在动，而客户以为已经放弃了。

### 7.4 状态 / 感知

```cpp
// 状态：既能"读快照"，也能"问一次"
std::optional<RobotStatusPtr> status() const;                      // 最新快照
Result<RobotStatusPtr> waitForStatus(Duration timeout);            // 等一条来
Result<RobotStatus> queryStatus(Duration timeout);                 // 服务：问一次（按值）
bool isHealthy() const;      // 缓存里**最新那条**健康吗（无故障 + 不在错误态 + 无急停）
float batteryPercentage() const;                                   // 读消息里的电量字段

// 感知：都是"最新一帧"语义，不是消息队列
std::optional<LaserScanPtr> laserScan() const;
Result<LaserScanPtr> waitForLaserScan(Duration timeout = Duration{0.0});  // 0 = 用配置里的超时
double nearestObstacleDistance(
    double angle_min = -std::numeric_limits<double>::infinity(),
    double angle_max = std::numeric_limits<double>::infinity()) const;    // 没有数据 → 返回 inf
```

### 7.5 结果对象：`Result<T>`

```cpp
bool ok() const;                          // 第一层：成功了吗
const Error& error() const;               // 失败原因（error().toString() 给可读文字）
T& value();                               // 取值（失败时抛 SdkException —— 安全网，不是常规通道）
const T& valueOr(const T& fallback) const;// 失败时给兜底值
```

> **SDK 的公开接口从不抛异常** —— 你必须显式检查 `ok()`。

---

## 8. 名词速查

| 名词 | 通俗解释 |
|---|---|
| **节点** | 一个参与通信的程序单元。一个进程里可以有多个 |
| **话题** | 一对多的"广播"：谁想听就订阅 |
| **服务** | 一问一答：你问，对方答，就一次 |
| **动作** | 长耗时任务：**能中途报进度、能被取消** |
| **消息** | 上面三种传的东西（有固定字段格式） |
| **回调** | 有东西到了时，系统调用的那个小函数 |
| **执行器** | 管"哪条线程跑哪个回调"的调度器 |
| **spin** | "让执行器转起来" —— 也就是本文的"值班" |
| **通信质量** | 可靠/尽力、队列多深…。**两边不一致会静默收不到** |
| **坐标变换** | "A 坐标系里的点，在 B 坐标系里是多少" |
| **服务端 / 客户端** | 动作（或服务）的**提供方** / **使用方** |

---

## 9. 五个常见误解

| 误解 | 事实 |
|---|---|
| **"spin 让机器人动起来"** | ❌ 它只传消息。机器人动由你发的**目标**决定 |
| **"spin 里在算什么"** | ❌ 什么也不算，纯粹"等别人给东西"（mock 里的控制回路才是"算"） |
| **"忘了 spin 只是慢一点"** | ❌ 是**完全收不到** —— 所有等待都会超时 |
| **"spin 就一个线程"** | ❌ 底下是**线程池**，默认 CPU 核数那么多个 |
| **"流程成功就是到了"** | ❌ 那只说明流程走完了，见 §6 |

---

## 10. 一句话总结

> **建机器人对象 = 建好通信管道；spin = 让管道通水；主线程专心写业务。**
>
> spin 是**泵**，不是**管道**；它不派活，它让工人上班。
> 你只需要记住：**发出去不用它，收回来全靠它。**
