# robot_sdk_tests

跨包的**验收探针**：验的是**行为**（忙时拒收、取消、折角、避障预检、加速度生效），
不是单个函数 —— 那些在 `robot_sdk_core` 的单元测试里。

放在仓库里的意义是**可复核**：谁都能跑一遍，不用再靠临时写一段。

## ⚠️ 跑之前先确认**没有游离的机器人**

**症状**：探针挂住（或报"等不到服务端"）—— 看着像**对面崩了**，**其实对面好好的**。

**真因**：**同名 action 服务端不止一个**。这时客户端会收到**不属于自己的** goal/cancel 回应，
rclpy 忽略它们 —— 但在某些交错下，**该来的那个也一起丢了**，于是那个 future **永远不完成**。
证据是 rclpy 自己打的那行警告（很容易被刷过去）：

```
Ignoring unexpected cancel response. There may be more than one action server
for the action 'move_to_joint_positions'
```

**【量】** 用 gdb 守着**两个 mock** 跑完整个序列：**一个都没崩**，四个进程全程活着；
但 `probe_motion` 和 `probe_wrap` 都把上面那行警告打了出来。所以"挂死"要怪**服务端不止一个**，
不是"谁死了"。

```bash
ros2 node list          # 只该看到你要的那几个
ros2 action list        # 或者看同名 action 有没有两个来源
```

### 为什么游离 mock 那么容易攒下来（**我踩了两轮才发现**）

**`ros2 run xxx &` 的 `$!` 是 `ros2 run` 那层 Python 壳，不是那个可执行文件。**
杀掉 `$!` 之后，里面的进程被 init 收养（`ppid` 变 1）**继续活着**。【量】实测：

```
$! = <python3 .../ros2 run ...>          ← 壳
真正的可执行文件是它的子进程
kill -9 $! 之后 → 可执行文件还在，ppid 变成 1
```

所以清理要么**直接起可执行文件**（`install/<pkg>/lib/<pkg>/<exe>`），要么按**可执行文件自己的 PID** 杀：

```bash
ps -eo pid,cmd | grep '[m]ock_robot_node' | awk '{print $1}' | xargs -r kill
```

> 探针每个等待都有 30 秒上限 —— 注意 `spin_until_future_complete` 超时**不代表就安全了**：
> rclpy 的 `Future.result()` 是**阻塞**的，future 没完成的话照样挂下去。
> 所以超时必须**明确收工**：`wait()` 抛 `TimeoutError`，`main()` 把它渲染成一行 + 退出码 1，
> pytest 那边则是一个 failure。
> （每个探针的 `wait()` 里就这四行。）

## 怎么跑

判定**只有一条路**：各探针里的 `check()` 是不符合预期就 `assert` 的本体，由 pytest 执行。

```bash
colcon test --packages-select robot_sdk_tests
colcon test-result --test-result-base build/robot_sdk_tests

# 或者只跑一条，并把探针打印的东西显出来（-s）：
pytest src/robot_sdk_tests/test/test_probes.py -s -k motion
```

**mock 是 fixture 自己起的**（`test/conftest.py`）—— 不用再开另一个终端。
顺手也把那个病根治了：fixture **直接起可执行文件**，`terminate()` 杀得干净。

```bash
$ colcon test && colcon test-result
Summary: 69 tests, 0 errors, 0 failures, 0 skipped        # 65 个单元测试 + 4 个探针
```

**为什么走 pytest**：`errors` 和 `failures` **天生分开** —— "环境没起好"和"行为不对"
一眼分得清，不用自己约定退出码。

| 文件 | 打谁 | 几条 |
|---|---|---|
| `test/test_probes.py` | mock 机器人 | 4 |
| `test/test_adapter.py` | 假下游 + 适配器 | 3 |
| `test/test_navigation.py` | Nav2 整栈 | 1 |

| 探针 | 验什么 |
|---|---|
| `probe_motion` | 忙时拒收 + 取消 |
| `probe_limits` | 超限的目标**当场被拒** |
| `probe_avoid` | `avoid_obstacles` 的两种取值 |
| `probe_accel` | `max_acceleration` 真的生效 |
| `probe_navigate` | 到得了 + 真的走了 + 取消得掉 |

## 适配器那条路

`test_adapter.py` 打的是**适配器**：先起"假下游 + 适配器"
（`launch/motion_chain.launch.py`），再对着 `/sdk/move_to_joint_positions`
跑**同一个** `check()`。

```bash
pytest src/robot_sdk_tests/test/test_adapter.py -s
```

`probe_avoid` 不在这组 —— 适配器只管关节运动，导航链路不在 `motion_chain` 里。

## 导航那条路（Nav2 整栈）

`test_navigation.py` 打的是 `ros2 launch robot_sdk_nav2 navigation.launch.py` 起来的
**整栈**：mock 机器人 + `nav2_adapter` + Nav2（planner / controller / behavior /
bt_navigator + map_server）。也就是说这一条同时压到：适配器的翻译、Nav2 的规划与追踪、
map_server、生命周期管理 —— 全仓最复杂的一条链路。

```bash
pytest src/robot_sdk_tests/test/test_navigation.py -s
```

它验四件事：**到得了**（`success`）、**误差在容差内**（≤ `xy_goal_tolerance` 0.15）、
**真的走了**（从 TF 上读 `map→base_footprint`，走了 1 m 以上才算）、**取消得掉**。

两处讲究：

- **先回原点**：不然"走了多远"取决于上一摊把机器人留在哪（`probe_accel` 栽过一次）；
- **就绪条件不是"服务端在"，也不是"代价地图有帧"**：Nav2 的动作服务端在节点构造时就
  advertise 了，那**不代表**生命周期激活完成；而**代价地图的帧是 latch 的**
  （`TRANSIENT_LOCAL`），它早于全栈就绪就已经存在了 —— 拿它当信号会**提前放行**，
  探针就会对着一个还在 activating 的 `bt_navigator` 发目标
  （实测：`Action server is inactive. Rejecting the goal.` → 整栈 abort）。
  **真判据是问生命周期**：`/bt_navigator/get_state` 到 `active` 才算"能干活"。
  （另一头也踩过：用默认的 `VOLATILE` QoS 订 latch 话题，会**一帧都收不到**。）

整栈有十来个节点，日志能把 pytest 的输出淹掉，所以 fixture 把它**写进临时文件**，
真出事时 assert 的消息里带着那个路径。

### 几条路**互斥**（靠 scope 保证）

三个 fixture 都是 **`module` 级** —— 每个文件各起一份，**拆干净了才走下一个文件**，
所以它们在时间上不重叠。

**别改成 `session` 级**：那样几个会同时活到会话末尾，而**同名 action 服务端不止一个**
时回应会串、future 可能**永远不完成**（见开头那节）—— 症状是"探针挂住，对面却好好的"。

> 这条以前是靠 `conftest.py` 里一个 `ACTIVE_ROUTE` 标志去拦的。后来发现它只挡得住
> 两件事：有人把 scope 改错、有人在一个文件里同时要两条路 —— 前者写在这儿就够了，
> 后者按现在的结构压根发生不了。**不为未来的事留冗余代码**，删了。

**收尾的兜底留着**（那些是当下就成立的，不是为将来）：
- `wait` 超时就往下 `kill` —— **拆卸不许抛**：抛了报告里多一条 error，进程还可能没死；
- 链路那条用 `os.killpg`（Popen 时 `start_new_session=True`）：`ros2 launch` 底下挂着
  假下游和适配器，只 kill 它自己会把孩子们变孤儿 —— 就是那个老病根。

## 限位归**执行侧**管

`probe_limits` 验的是"**超限的目标该被当场拒**"。判这个的是**机器人自己**（mock ✅，
真机上将来是控制器/驱动器 ✅）—— 它从自己的 URDF 里读限位，目标一到就判。

**SDK 不查这个** ✅：它硬件无关，"这关节能转到哪"不是它该记的事 —— 两处各存一份规则
迟早漂移。所以 fixture 起 mock 时要把描述喂给它：

```
--ros-args -p robot_description_file:=<install/robot_description/share/.../mobile_manipulator.urdf>
```

不喂的话 mock 不认识限位（那正是它以前的行为：超限目标照发照转）。
