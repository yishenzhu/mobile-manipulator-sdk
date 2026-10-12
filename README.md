# robot_sdk

一个 ROS 2（Jazzy）移动操作机器人 SDK 的参考工程。
**目标**：客户代码只依赖一套稳定的 SDK 契约，换后端（假机器人 / Gazebo / 真机）**一行不用改**。

```
        客户代码（robot_sdk_examples / robot_sdk_py）
                │  只用 robot_sdk/ 的头文件
                ▼
        ┌───────────────┐
        │ robot_sdk_core │  ← 契约：类型 / 配置 / Robot 门面 / 4 个能力 API
        └───────┬───────┘
                │  DDS（话题 / 服务 / action），契约的两侧可互换
     ┌──────────┴──────────┬──────────────────┐
     ▼                     ▼                  ▼
 robot_sdk_sim        robot_sdk_gazebo   真机（未实现）
 （假机器人，默认）      （Gazebo 仿真）
     ▲
     │  适配层把"别的栈"翻译成 SDK 契约
 robot_sdk_bridge ──▶ Nav2 / ros2_control / MoveIt
```

即使你只想跑个 demo 不关心仿真，也建议先看 **《5 分钟跑通》**——全程不需要 Gazebo 和真机。

---

## 5 分钟跑通

### 0. 依赖

需要 ROS 2 **Jazzy**、`colcon`、`rosdep`。首次在别的机器上构建时：

```bash
sudo rosdep init && rosdep update          # 只做一次
cd <这个仓库根目录>
rosdep install --from-paths src --ignore-src -r -y
```

基础路径（跑 mock + 示例 + 测试）只需要 `rclcpp` 系列。
**进阶路径**额外需要：`nav2_bringup`（导航整栈）、`ros_gz_sim` / `gz_ros2_control`（Gazebo）、
`moveit` / `moveit_configs_utils`（MoveIt）、`pybind11`（Python 绑定）。

### 1. 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.sh          # 每个新终端都要 source 一次
```

> 只改某个包时可以 `colcon build --symlink-install --packages-select robot_sdk_core`。
> 仓库里的 `.localrc` 攒了几个别名：`cbd` = build、`rup` = source install、`cst` = test、`ces` = test-result。

### 2. 起假机器人（终端 1，长驻）

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.sh
ros2 run robot_sdk_sim mock_robot_node
```

它一个人提供全部契约：状态话题 + 状态服务 + 关节话题 + TF + 激光/IMU/相机/点云 +
两个动作服务端（导航、关节运动）。**不需要 Gazebo。**

### 3. 跑第一个示例（终端 2）

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.sh
ros2 run robot_sdk_examples hello_robot
```

实测输出（正前方 4.90 m 是解析解，房间 x ∈ [-5, 5]，激光装在 base 前 0.1 m）：

```
=== Robot SDK hello ===
状态  : state=1  电量=95.7%  充电中=0
健康  : 1
服务  : 已运行 8.7 秒
激光  : 360 条射线，正前方最近障碍 4.90 m
IMU   : 角速度 z = 0.000 rad/s
相机  : 深度图 64x48 (32FC1)
相机  : 点云 64x48
位姿  : x=0.00  y=0.00  yaw=0.00
```

---

## 跑其余的示例

四个 C++ 示例都是"客户代码长什么样"，都只依赖 `robot_sdk_core`。**都在终端 2 跑**，
前提是终端 1 的假机器人还活着。

| 示例 | 干什么 | 命令 |
|---|---|---|
| `hello_robot` | 最小可用：读状态 + 全部传感器 + 查位姿 | `ros2 run robot_sdk_examples hello_robot` |
| `demo_navigation` | 边走边看反馈 + 一次阻塞调用回原点 | `ros2 run robot_sdk_examples demo_navigation` |
| `demo_motion` | 关节运动 + 夹爪开合 | `ros2 run robot_sdk_examples demo_motion` |
| `demo_manipulation` | 具名位姿 → 夹爪 → home | `ros2 run robot_sdk_examples demo_manipulation` |

### ⚠️ 接适配器时，动作名要换

示例的**动作名是可配的第一个位置参数**，因为同一个动作名不能有两个服务端：

```bash
# 对着 mock 直接打（默认名）
ros2 run robot_sdk_examples demo_navigation

# 对着 Nav2 整栈打（适配器占的是 sdk/ 前缀）
ros2 run robot_sdk_examples demo_navigation sdk/navigate_to_pose

# 关节运动同理：接运动适配器时
ros2 run robot_sdk_examples demo_motion sdk/move_to_joint_positions
```

### Python 示例

Python 和 C++ 调的是**同一份核心**（pybind11 绑定），行为与错误语义一致。见
[`src/robot_sdk_py/README.md`](src/robot_sdk_py/README.md)。

```bash
python3 $(ros2 pkg prefix robot_sdk_py)/share/robot_sdk_py/python/hello_robot.py
python3 $(ros2 pkg prefix robot_sdk_py)/share/robot_sdk_py/python/demo_navigation.py
```

---

## 跑测试

```bash
colcon test                                   # 全量
colcon test --packages-select robot_sdk_tests  # 跨包验收探针
colcon test-result --verbose
```

`robot_sdk_tests` 里的探针是**行为验收**（忙时拒收、取消、限位、避障预检、加速度、真栈解姿态），
不是单元测试。mock 由 fixture 自己起 —— **不用另开终端**。详见
[`src/robot_sdk_tests/README.md`](src/robot_sdk_tests/README.md)。

> Gazebo 那条**故意不进 `colcon test`**：headless 全套要 ~1.1 GB 内存、约 3 分钟，
> 且精度依赖物理，当 CI 判据又慢又抖。

---

## 包一览

| 包 | 角色 |
|---|---|
| `robot_sdk_core` | **SDK 契约 + 客户端库**：`robot_sdk/types.hpp`、`config.hpp`、`Robot` 门面、4 个能力 API |
| `robot_sdk_interfaces` | 自定义 msg / srv / action 定义 |
| `robot_sdk_sim` | **假机器人**（`mock_robot_node`）+ 假下游（`joint_trajectory_mock` / `move_group_mock`） |
| `robot_sdk_bridge` | 适配层：`nav2_adapter` / `motion_adapter` / `moveit_adapter` |
| `robot_sdk_nav2` | Nav2 整栈 launch + 地图 + 参数 |
| `robot_sdk_gazebo` | Gazebo 仿真 + gz↔ROS 桥 + odom TF |
| `robot_description` | URDF / SRDF / 运动学 / 控制器配置；`ros2_control`、`move_group` 的 launch |
| `robot_sdk_examples` | 4 个 C++ 示例（客户代码） |
| `robot_sdk_py` | Python 绑定（pybind11） |
| `robot_sdk_tests` | 跨包验收探针（pytest） |

---

## 文档地图

| 想了解 | 看 |
|---|---|
| **哪个文件装了什么、谁调用谁**（10 个包的文件地图 + 接线总表 + 找功能索引） | [`docs/file-map.md`](docs/file-map.md)（**想改代码时先看这份**） |
| **机器人是怎么动起来的：业务流转全图，mock / Gazebo / 真机三条路对照** | [`docs/robot-flows.md`](docs/robot-flows.md)（想懂行为先看这份） |
| **SDK 怎么被调用、`spin()` 在干什么、和手写 ROS 2 有何不同** | [`docs/sdk-call-flow.md`](docs/sdk-call-flow.md)（偏入门） |
| mock 的**内部设计**（并发模型、动作骨架、传感器世界） | [`docs/mock-robot-node.md`](docs/mock-robot-node.md) |
| **怎么看机器人在哪 / 3D 空间；URDF 怎么发布和被谁用** | [`docs/visualization.md`](docs/visualization.md) |
| 逐次踩坑的历史与推导过程 | [`学习笔记.md`](学习笔记.md) |
| Python 绑定的边界约定（异常 / GIL） | [`src/robot_sdk_py/README.md`](src/robot_sdk_py/README.md) |
| Gazebo 那条链路怎么起、已知坑 | [`src/robot_sdk_gazebo/README.md`](src/robot_sdk_gazebo/README.md) |
| 验收探针怎么跑、为什么老挂住 | [`src/robot_sdk_tests/README.md`](src/robot_sdk_tests/README.md) |

还没建立全局印象时，先看 [`docs/file-map.md`](docs/file-map.md)：它按包列出**每个文件干什么**，
并把包之间的调用关系画成图。

**建议的读代码顺序**：`robot_sdk_core/include/robot_sdk/config.hpp`（契约是什么）
→ `robot_sdk_sim/src/mock_robot_main.cpp`（几条"车道"）
→ `robot_sdk_sim/include/robot_sdk_sim/mock_robot_node.hpp`（类结构与纪律）
→ `robot_sdk_core/include/robot_sdk/detail/action_slot.hpp`（动作骨架）。

要看类图 / 调用图，仓库根有个 `Doxyfile`（复用 `build/compile_commands.json`）：

```bash
doxygen Doxyfile && xdg-open build/doxygen/html/index.html
```

---

## 进阶：真 Nav2 驱动这个机器人

把 mock + Nav2 + 适配器一起拉起来（最复杂的一条链路，验的是整栈）：

```bash
ros2 launch robot_sdk_nav2 navigation.launch.py        # 终端 1
ros2 run robot_sdk_examples demo_navigation sdk/navigate_to_pose   # 终端 2
```

Gazebo 里跑参考机器人（手臂由 ros2_control 驱，MoveIt 那条链路也走得通）：

```bash
ros2 launch robot_sdk_gazebo gazebo.launch.py
ros2 launch robot_sdk_gazebo gazebo.launch.py headless:=true   # 无显示器
```

详细的搭配与已知问题见各自的 README。

**想直接看机器人在哪、看 3D 空间**（RViz / Gazebo / 没有图形环境时的办法，以及 URDF 是怎么
发布和被谁用的）：见 [`docs/visualization.md`](docs/visualization.md)。最快的一条：

```bash
ros2 launch robot_sdk_sim mock_robot.launch.py rviz:=true    # mock + rviz2（需要图形环境）
```

---

## 常见问题

**探针挂住 / 报"等不到服务端"，但对面明明活着。**
多半是**有游离的 mock 残留**：同名动作服务端不止一个时，客户端会收到不属于自己的回应，
某些交错下该来的那个也一起丢了。先检查：

```bash
ros2 node list
ros2 action list
ps -eo pid,cmd | grep '[m]ock_robot_node' | awk '{print $1}' | xargs -r kill
```

> **注意**：`ros2 run xxx &` 时 `$!` 是 `ros2 run` 那层 Python 壳，**杀掉它里面的可执行文件会
> 被 init 收养继续活着**。要清理请直接按可执行文件自己的 PID 杀（如上），或直接起
> `install/<pkg>/lib/<pkg>/<exe>`。

**终端里 `source: not found`。**
你可能在 `sh` 里跑。ROS 的 setup 脚本要用 **bash**：`bash -c 'source ... && ...'`，
或干脆 `bash` 进一个交互式 shell。
