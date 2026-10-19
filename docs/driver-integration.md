# 驱动接入：一页清单

> 目标：把"厂家驱动 → 本 SDK"接起来，并且**当场能判断接对了没有**。
> 两类驱动的判据**不同**：传感器（话题型 ✓）见第二节，执行器（ros2_control 型 ✓）见第四节。
> 配套工具：`scripts/check_driver_topic.py` ✓ —— 六项判据一条命令全查。

## 一、先分清两类

| | 传感器型 ✓ | 执行器型 ✓ |
|---|---|---|
| 例子 | 雷达 / 相机 / IMU | 底盘电机 / 机械臂关节 |
| 接口 | **话题**（pub/sub）| **ros2_control 接口**（`read()`/`write()`）|
| 为什么这么分 | 单向、高频、可丢 ⇒ 用 QoS 决定丢谁 | 需**同拍反馈**且**独占**（一个关节只能一个写者）|
| 本文哪节 | 二、三节 ✓ | 四节 ✓ |

## 二、传感器型：六项判据（每项都能一条命令复查 ✓）

| # | 判据 | 为什么 | 怎么查 / 判定 |
|---|---|---|---|
| 1 | **发布者与类型** | 没发布者时后面全无意义 | `ros2 topic info -v /imu` ⇒ `Publisher count ≥ 1`；类型冲突 ⇒ ✗ |
| 2 | **QoS 兼容** | 不兼容时**一条数据都收不到**，且不报错 ✗ | 体检脚本 ✓。`best_effort` 发布者 ⇒ **reliable 订阅者收不到**；`transient_local` ⇒ volatile 拿不到历史；**同一话题多发布者 QoS 不同 ⇒ 订阅侧按最弱配** |
| 3 | **频率与抖动** | 算法按声明频率设计 | `ros2 topic hz /imu`；与声明偏差 >30% ⇒ ✗（本仓实测过 `/imu` 200 Hz vs 声明 100 ✗）|
| 4 | **时钟源** | 混用 sim/墙钟 ⇒ TF 查不到、超时判定全错 ✗ | 有 `/clock` 发布者 ⇒ 全体必须 `use_sim_time:=true`；否则用墙钟 |
| 5 | **`frame_id` 在 TF 树里** | 拼错就整条 TF 链断，消费方投影全错 | 体检脚本 ✓（`base_footprint → <frame>` 查询）|
| 6 | **端到端延迟** | 只在**同钟源**时有意义 | 体检脚本 ✓；sim 时间下跳过（墙钟差值无语义）|

## 三、没有真硬件时怎么验证（三招，按可信度排序）

1. **`rosbag2` 回放** ✓ —— 把真实驱动的输出录下来再放，SDK 侧看到的与真机一致：
   `ros2 bag record -o real_driver /scan /imu /camera/depth/image_raw` ⇒ `ros2 bag play real_driver`
2. **假发布者 + 故障注入** ✓ —— 故意做坏：QoS 错 ✓ / `frame_id` 错 ✓ / 频率掉到 1/10 ✓
   ⇒ **体检脚本必须报红** ✓。这条是"可观测性"唯一的硬证据 ✓。
   现成脚本：`./scripts/run_fault_injection.sh`（它会造一棵最小 TF 树，保证每类**只点亮一项**）：

   | 注入的故障 | 体检脚本的判定 |
   |---|---|
   | ① QoS：`best_effort` 发布者 | ⚠️ 发布者是 best_effort ⇒ reliable 订阅者收不到（✗ 0 ✓）|
   | ② `frame_id: no_such_frame` | ✗ TF `base_footprint → no_such_frame` 查不到（✗ 1）|
   | ③ 实际 2 Hz（声明 10） | ✗ 频率对齐：实测 2.2 vs 10.0（偏差 78%，✗ 1）|
   | ④ 时间戳固定在 1970 | ✗ 时钟源：与本机差 +1.79e9 s，且没有 `/clock`（✗ 1）|

   ⚠️ 用 `ros2 topic pub` 造数据时它**不填 `header.stamp`**（实测 `sec=0`）⇒ ①②③ 用
   `--ignore-clock` 把时钟源那项显式跳过，否则每类都会连带红一项、证据就混了。
3. **`ros2_control` 的 `mock_components`** ✓ —— 占住"硬件插件"那个位置 ✓
   ⇒ 证明上层只依赖**接口**、不依赖实现 ✓（本仓**依赖 ros2_control 的那几条测试**就跑在这条上 ✓；
   另有一批用例用的是"假服务端"而不经过 ros2_control ✓）

⚠️ 这三招能证明：接口/流程/可观测性 ✓。**不能**证明：真实时序抖动 ✓ 编码器噪声 ✓ 驱动错误码语义 ✓ 传感器畸变 ✓。

## 四、执行器型：`ros2_control` 硬件插件

- 它是什么 ✓：实现 `SystemInterface` 的 `.so` ✓（`read()`/`write()` 是纯虚接口，见
  `hardware_component_interface.hpp`）；URDF 里 `<hardware><plugin>` 指定用哪个。
- 本仓现在占位的是 ✓：`gz_ros2_control/GazeboSimSystem`（仿真）✓、`mock_components/GenericSystem`（测试）✓
- **换真机只改这几处** ✓：
  1. **URDF** 的 `<plugin>` 一行 ⇒ 厂家插件
  2. **控制器参数**（`ros2_control.yaml`）：轮距/半径 / 限位 / 关节名
  3. **进程位置** ⚠️：gz 插件跑在 gz 进程里；真机插件要跑在 **`ros2_control_node`**
     ⇒ 会**多一个进程**（这是仿真与真机唯一的结构性差异）
  4. **接口必须对得上** ⚠️：轮子给 `velocity` 命令 + `position`/`velocity` 状态；
     若厂家只给 `effort`、或关节名不同 ⇒ 改 URDF/参数（**客户接口仍然不变** ✓）
- 判据 ✓：控制器 `list_controllers` 全 `active` / 闭环频率（`controller_manager.update_rate`）/
  命令 → 状态跟随 / 失败态（错误码、急停）。

## 五、本仓实测过的三个例子（都是"静默"的）

1. **`/imu` 200 Hz，而 URDF 声明 100** —— 传感器**系统**被声明了两次（world ✓ + URDF ✓），
   两个实例各发一遍。修法：系统归 world、传感器本体留在 URDF。**是体检脚本抓到的** ✓。
2. **`/joint_states` 有两个发布者** —— gz 的 `JointStatePublisher` 只发车轮、`joint_state_broadcaster`
   只发手臂 ⇒ 两套数据（~1 kHz 与 50 Hz）。修法：JSB 发全部 9 个关节，删掉 gz 侧与那条桥。
3. **`/cmd_vel` 的类型** —— Jazzy 的 `diff_drive_controller` **只收 `TwistStamped`** ⇒
   nav2 侧开 `enable_stamped_cmd_vel`、mock 同步改；同时确认 `cmd_vel` 不属于 SDK 契约。

## 六、验收清单（照着打勾）

- [ ] `python3 scripts/check_driver_topic.py <topic>` 全 ✓
- [ ] TF **单一来源**（每条边只一个作者）
- [ ] `/joint_states` **单一发布者**（`ros2 topic info -v` 数发布者）
- [ ] `ros2 control list_controllers` 全 `active`
- [ ] 命令 → 里程计 / 关节状态 一致（不是"服务端说成功"就算）
- [ ] 故障注入（QoS / 帧 / 频率）能被报警
- [ ] **把没测的写出来**（真机时序、噪声、错误码）

## 七、不做（诚实边界）

没有真机 ⇒ 物理层（时序抖动、编码器噪声、驱动错误码语义、传感器畸变）**一律不声称**，
只在真正接入后补实测数字。**可讲的**是：换插件的位置、判据、工具、以及"静默失败"的排查链。
