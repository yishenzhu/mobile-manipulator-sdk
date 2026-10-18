# Changelog

记录**契约面**的变化（ROS 接口 / C++ 公开头 / Python 面 / MCP 工具面 / 配置面）。
格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，版本号遵循[语义化版本](https://semver.org/lang/zh-CN/)。
"只增不删"的政策、什么算破坏、发版步骤：见 [`docs/contract.md`](docs/contract.md)。

## [Unreleased]

### 改了（破坏性 ⚠️）

- **`Robot::spinSome(timeout)` → `Robot::spinOnce(timeout)`**（Python：`spin_some` → `spin_once`）。
  原名字借的是 ROS 的 `spin_some`（*= 把**当前就绪**的项全部干完、立即返回*），但实现是
  `executor_->spin_once(timeout)`（*= 最多等 `timeout`、**只干一个***）—— **名字和行为不一致** ✗。
  改名后与 rclcpp 的 `spin_once` / rclpy 的 `spin_once` 对齐 ✓。**语义与实现未变** ✓。
  ⚠️ 破坏性：C++ 与 Python 的公开名字都变了 ⇒ 版本 **0.3.0 → 0.4.0**。

### 加了（新能力）

- **唯一性守卫**：mock 默认只允许「本机 + 本域」跑一个实例（`$XDG_RUNTIME_DIR` 下的 `flock` 锁文件；
  锁打不开即拒绝启动，退出码 1）。同域要并存加 `--ros-args -p unique:=false`。边界与取舍见 `docs/improvements.md`。
- mock 发布 `/robot_description`（latch），并把轮子关节并入 `/joint_states` ⇒ RViz 里轮子/底盘的 TF 完整。

### 修了（缺陷修复）

- **mock 侧的 `/joint_states` 有重复发布者** ✗：`mock_robot_node`（机器人本体）与 `joint_trajectory_mock`
  （假 JTC）**都发** ⇒ 同起时下游看到两套数据（与 gazebo 侧修掉的是同一类问题 ✓）。对齐业界口径
  （一个栈一个发布者 ✓；多来源要用命名空间 ✓）：给 `joint_trajectory_mock` 加参数
  **`publish_joint_states`，默认 `false`**，只有"没有 mock 节点"的链路（`motion_chain.launch.py`）
  由 launch 显式打开 ✓。**实测**：两栈同起时发布者都只有 **1** 个 ✓（A：`joint_trajectory_mock`；
  B：`mock_robot`）✓。
- **`stop()` 早于 `spin()` 时被吃掉 ⇒ `spin()` 永久阻塞**（契约写的是"直到 `stop()` 被调用"）。
  根因是 rclcpp 的**已知竞态**（ros2/rclcpp#3172、#1454）：`stop()` 只是 `executor.cancel()`，而 `spin()`
  入口会 `spinning.exchange(true)` **无条件**覆盖那个状态。上游只在 **rolling** 修了、且改动了类布局
  ⇒ jazzy/kilted/lyrical 都拿不到（各分支源码逐一核对过）⇒ SDK 侧自兜：`RobotRuntime::stop_requested_`。
  语义与上游修法一致（**意图只消费一次**）：`stop(); spin()` 立即返回 ✓，之后还能再 spin() ✓。
  **实测**：删掉 `test_robot_lifecycle.cpp` 里那句 workaround sleep ⇒ 立刻挂死（exit 124，并连带报
  "guard condition: the given context is not valid"）；装上守卫后 `StopBeforeSpinIsHonored` 68 ms 返回 ✓；
  撤掉守卫 ⇒ 两条新用例如实报红（~2 s）且**不挂死** ✓。
- **`MotionAPI` / `NavigationAPI` 不持有传入的回调组 ⇒ action 的响应收不到**。
  两者只把 `callback_group` 透传给 `create_client`，自己不保存；而 `node->create_callback_group()`
  返回的组，**节点只存 `weak_ptr`**（rclcpp 的 `node_base.hpp`）⇒ 调用方一旦**内联**传临时对象，
  组会随整条语句结束析构 ⇒ 挂在它上面的订阅再也不会被 executor 处理。
  症状很隐蔽：服务端**确实接受**了目标（`goal_callback` / `accepted_callback` 都跑了），但客户端的
  **goal response 永不回来** ⇒ `goal_future_` 不兑现 ⇒ 派发等满 `server_wait_timeout_s` 报 `kUnavailable`。
  **实测**（同一条用例内逐位二分 + 双向干预）：内联传组 ⇒ 失败 3/3；存成变量 ⇒ 成功 3/3；
  加上持有 ⇒ 内联也成功，**撤掉又失败**（可逆 ⇒ 不是构建假象）。
  修法与 `status_api` / `transform` 一致：各持有一个 `callback_group_` 成员。
  回归用例：`MotionApiTest.InlineCallbackGroupStillDispatches` + `NavigationApiTest.InlineCallbackGroupStillDispatches`
  （撤掉修复时两条都会红，已验证）。
  ⇒ **`PerceptionAPI` 也照同一写法持有了**，但**实测它的订阅并不受这个问题影响**（撤掉修复后
  感知用例照样过 ⇒ 因此**没有**为它写回归用例，避免留下"抓不住东西的空炮"）。改动只为三者一致。
- mock 的到位判据原先只看 xy、朝向又直接赋值 ⇒ **原地转身在 RViz 上是瞬移**；现改为限速转向 + 位姿共同判定。
- mock 在 `remaining` 为 0（原地转身）时除零 ⇒ NaN 写进位姿，此后 **所有 TF 被 RViz 丢弃**（模型/轮子/深度图全红）；已加除零保护。
- 可视化：RViz 缺 TF（相机/激光/底盘链）、`black` 材质重复定义、MockRobotModel 默认未启用等一串问题修复。
- **传感器帧重复发布**：mock 与 URDF 各发一套挂载 TF ⇒ 同一个 `imu_link`/`camera_link` 有两个父节点、
  `TF_REPEATED_DATA`、位置在两个值之间跳；已统一到 URDF（`/tf_static` 发布者 2 → 1）。
- **`cancel()` 在"目标已发出、还没被接受"的窗口里被静默丢弃**：那一刻取不到 goal handle，原先直接 `return`，
  连"客户要过取消"这个意图都没记下 ⇒ 目标随后被接受并**照常执行**，谁也停不了。
  实测（真 mock）：派发后立刻 `cancel()`，**12/12 次取消都被丢掉**（终态 `SUCCEEDED` ✗）。
  按业界做法（nav2 的 `BasicNavigator`）把**窗口本身消掉**，而不是绕过它 —— 派发时就等到"接受"，
  没等到**就不把 handle 交出去**（`return nullptr`，与 nav2 的 `if not accepted: return None` 同构），
  之后 `cancel()` 一定有 goal handle 可用。受害面是"刚派发就反悔"的路径
  （`cancelAll()`、agent 超时、用户按停）—— 恰恰是最需要取消的时刻。

### 变了（行为变化）

- **⚠️ 破坏性：三个 `*Async` 的返回可能为**空**。** `goToAsync` / `moveToJointPositionsAsync` /
  `moveToPoseAsync` 现在等到"接受"再返回；**超时未被接受、或被拒收 ⇒ 返回空指针**（Python 侧是 `None`）。
  旧代码 `nav.goToAsync(g)->wait(...)` 必须改成**先判空**，否则空指针崩 ✗。
  对应地，三个同步版（`goTo` / `moveToJointPositions` / `moveToPose`）在派发阶段失败时返回
  `ErrorCode::kUnavailable`；⚠️ **这与"服务端不在场"之外**的旧行为不同：以前那种情况是在**等待阶段**报 `kTimeout`。
  超时上限取自 `server_wait_timeout_s`（`MotionConfig` 补上该项，默认 5.0，与 `NavigationConfig` 同名同义）。
- **`ActionHandle::cancel()` 现在等服务端的取消响应，并返回 `bool`**（对齐 rclpy 的 `cancel_goal()`、
  nav2 的 `cancel_goal_async()` + 等待）：返回值 = 服务端是否受理（响应 `goals_canceling` 里列了这个 goal）。
  ⚠️ **不要从 executor 线程（某个回调里）调用** —— 等响应要靠 executor 处理，等下去会死锁
  （实测：在回调里改成等待后，用例永不返回，只能用 `kill -9` 收尾）。要确认**真的停下**仍看 `wait()` 的终态是否 `CANCELED`。
- **⚠️ 破坏性：`state` 单值枚举 → `state_bits` 位掩码（`uint32`）**。旧常量 `STATE_IDLE/NAVIGATING/MOVING/…` 全删，改为
  位：`bit0 STATE_BASE_MOVING`（底盘在动，**含 cmd_vel 路径**）、`bit1 STATE_ARM_MOVING`、`bit2 STATE_CHARGING`、
  `bit3 STATE_ESTOP`、`bit4 STATE_BATTERY_LOW`；同时删掉与之重复的 `is_charging` / `is_battery_low` / `is_estop_engaged`。
  ⇒ **旧单值装不下"两个独立运动通道"**（臂在动时底盘其实照常能发），而且旧版 `NAVIGATING` 漏掉了 `cmd_vel` 直接控速的情形。
- **新增判据（请用它，别自己位运算）**：`allowsMotion()` / `allowsNavigation()`（读缓存）与自由函数
  `motionAllowed(status)` / `navigationAllowed(status)`；**没有状态数据时一律 false**（fail-closed）。MCP 的
  `get_status` 返回里也带上 `state_bits` / `allows_motion` / `allows_navigation`。

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
