# 可视化：看机器人在哪、看 3D 空间

> 本文只讲**怎么看**，不讲 mock 内部怎么设计（那是 `mock-robot-node.md`）。
> 读完你应该能回答两个问题：**用哪个工具**，以及 **URDF 到底是怎么被发布和被谁用的**。

本文沿用全项目那条纪律：**【量】= 本机跑过，有数字；【推】= 从代码/文档推导，没实测**。

---

## 1. 先知道料在哪

mock 一个人就把要画的东西全发出来了（不需要 Gazebo）：

| 话题 | 类型 | 频率 | 在 RViz 里对应 |
|---|---|---|---|
| `/tf`、`/tf_static` | `tf2_msgs/TFMessage` | 50 Hz / 一次 | TF、RobotModel |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | LaserScan |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 10 Hz | PointCloud2 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 10 Hz | Image |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz | Camera（内参） |
| `/joint_states` | `sensor_msgs/JointState` | 50 Hz | RobotModel（关节角） |
| `/robot_status` | `robot_sdk_interfaces/RobotStatus` | 10 Hz | 无内置 display（用 Plot/rqt） |

**TF 树【量】**（起 mock 后 `ros2 run tf2_tools view_frames` 读出来的）：

```
map ──▶ odom ──▶ base_footprint ──┬──▶ laser_frame
                                  ├──▶ imu_link
                                  └──▶ camera_link ──▶ camera_optical_frame
```

`map → odom → base_footprint` 由 mock 直接广播 ⇒ **相当于"定位永远完美"**。所以 RViz 的
**Fixed Frame 用 `map`**，机器人就落在原点（【量】`tf2_echo map base_footprint` = `[0, 0, 0]`）。

---

## 2. RViz2：最直接的那个（本机已装）

### 2.1 起法

```bash
# 一条命令：mock + rviz（rviz:=true 是新加的开关，默认 false）
ros2 launch robot_sdk_sim mock_robot.launch.py rviz:=true
```

或者分开起（想单独调 rviz 时）：

```bash
ros2 run robot_sdk_sim mock_robot_node
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix robot_sdk_sim)/share/robot_sdk_sim/rviz/mock.rviz
```

配置就是仓库里的 `src/robot_sdk_sim/rviz/mock.rviz`（构建后装到 `share/robot_sdk_sim/rviz/`）。

> ⚠️ **RViz 需要图形环境**。SSH 进来时 `DISPLAY` 是空的，`rviz2` 会直接报
> `cannot open display`。两条出路见 **§5**。
>
> ⚠️ 在 RViz 里改完再保存会**重写这个文件并丢掉注释**。要留注释就在别处另存。

### 2.2 配置里已经放好了什么

| Display | 话题 | 默认 | 说明 |
|---|---|---|---|
| Grid | — | 开 | 地面参考 |
| TF | `/tf`、`/tf_static` | 开 | 先看它，树不对后面都不用看 |
| LaserScan | `/scan` | 开 | 四面墙的一圈点 |
| DepthPoints | `/camera/depth/points` | 开 | 深度相机的点云 |
| DepthImage | `/camera/depth/image_raw` | 开 | 深度图（0–5 m 归一化） |
| RobotModel | `/robot_description` | **关** | 见 §2.3，默认给不了 |
| Map | `/map` | **关** | 跑 Nav2 整栈时再打开（`map_server` 才有） |

### 2.3 两个必须知道的坑

**坑一：三个传感器订阅必须是 Best Effort。**

mock 用 `SensorDataQoS()` 发布（Best Effort / Volatile），RViz 的 display 默认可能是
Reliable —— **两边不匹配就静默收不到**，不报错、只是画面上没有点。配置里已经全设成
Best Effort 了（`Reliability Policy: Best Effort`），自己加 display 时要记得。

这和 SDK 侧 `perception_api.cpp` 踩过的是同一个坑（见 `mock-robot-node.md` §11.5）。

**坑二：RobotModel 和 mock 的传感器 TF 对不上。**

这是**实测**出来的，而且有三处不一致 —— 想看机器人本体之前必须先知道：

| | URDF（`robot_description`） | mock（`mock_robot_node.cpp:216-221`） |
|---|---|---|
| 激光帧名 | `laser_link` | **`laser_frame`**（URDF 里没这个名字） |
| 三个传感器的**父帧** | `base_link` | **`base_footprint`** |
| 激光挂载点 | `0.16, 0, 0.105`（相对 base_link） | `0.1, 0, 0.2`（相对 base_footprint） |
| IMU 挂载点 | `0, 0, 0.03` | `0, 0, 0.1` |
| 相机挂载点 | `0.2, 0, 0.11` | `0.2, 0, 0.3` |

（`base_link` 自己比 `base_footprint` 高 `0.055`，所以两边的 z 差还要再加上这 0.055。）

后果：**同时**起 mock 和 `robot_state_publisher` 时，`/tf` 和 `/tf_static` 各有
**2 个发布者【量】**，`imu_link` / `camera_link` 就有了**两个不同的父节点**，位置也不一样。
TF 会报 `TF_REPEATED_DATA`，传感器帧会在两个位置之间跳。

所以现实里是二选一：

- **只看 mock 自己的数据**（激光/点云/深度图 + 机器人位姿）—— 一切正常，`laser_frame` 的位置是对的
  （mock 的世界自洽，`mock-robot-node.md` §11.6 那套解析解就是照它的偏移验的）。
- **要看机器人本体** —— 模型来自 URDF，于是那几条传感器帧会和 mock 打架。

> 这两套数字是**各自独立定的**，从来没对齐过。要统一的话是个设计决定
> （改 mock 会动到 §11.6 那批已验收的解析解；改 URDF 会动到 MoveIt/Gazebo 那侧），
> 不在本文范围内。

### 2.4 真想看机器人本体（RobotModel）

mock **不发布** `robot_description`（【量】它的 publisher 列表里没有），所以三选一：

**① 最快：让 RViz 直接读文件。** 把 RobotModel 的 `Description Source` 改成 `File`，
`Description File` 填：

```
install/robot_description/share/robot_description/urdf/mobile_manipulator.urdf
```

（这个文件是**构建期**由 xacro 展开出来的，见 §4。`--symlink-install` 下它是指向 `build/` 的软链。）

**② 起 `robot_state_publisher`** —— 这样模型还能跟着 `/joint_states` 动。

⚠️ **不能用命令行 `-p` 传 URDF**：参数覆盖是按行解析的，多行 XML 会直接把节点打崩
（【量】`Failed to parse global arguments` → `RCLInvalidROSArgsError`）。必须走 launch 或
params 文件：

```python
Node(package="robot_state_publisher", executable="robot_state_publisher",
     parameters=[{"robot_description": open(urdf_path).read()}])
```

**③ 别用 RobotModel** —— 只看激光/点云/深度图 + TF 的坐标轴，很多时候就够了。

### 2.5 命名空间那条

```bash
ros2 launch robot_sdk_sim mock_robot.launch.py namespace:=robot1 rviz:=true
```

上面这个组合**画面上会是空的**：mock 的话题变成 `/robot1/scan`、`/robot1/tf`，而
RViz 的 TF display 订的是**绝对名** `/tf`（launch 里 rviz 也**故意不进命名空间** ——
它要看整张图）。要么别加 `namespace:=`，要么手工把每个 display 的话题前缀改成 `/robot1`。

---

## 3. Gazebo：完整的 3D 世界 + 物理

RViz 画的是**契约上的数据**；Gazebo 跑的是**真的物理仿真**，能看到机器人在一个房间里、
手臂由 `ros2_control` 真的驱动。

```bash
ros2 launch robot_sdk_gazebo gazebo.launch.py                 # 带 GUI
ros2 launch robot_sdk_gazebo gazebo.launch.py headless:=true   # 无显示器 / 省内存
```

| | RViz | Gazebo |
|---|---|---|
| 看什么 | SDK 契约上的数据（mock 发的） | 真实物理 + 渲染 |
| 谁发数据 | mock（假机器人） | gz 插件 + 桥 |
| 内存 | 小 | **大** |
| 用途 | 开发时看"客户能拿到什么" | 验"物理上跑不跑得动" |

> ⚠️ **内存是硬约束【量】**：这台机器 5.9 GB 总，headless 全套要 ~1.1 GB、约 3 分钟，
> 起栈后只剩 ~1 GB，再叠东西会被 OOM 杀掉（表现为整个栈莫名其妙消失）。
> 详见 `robot_sdk_gazebo/README.md`。

Gazebo 那条链路自己的坑（控制器激活、车轮关节状态、桥的时间戳）都在那个 README 里，不在这里重复。

---

## 4. URDF 是怎么发布、怎么被使用的

### 4.1 源头与构建期

**唯一的手改点是 `src/robot_description/urdf/mobile_manipulator.urdf.xacro`。**
纯 URDF 是构建时由它展开出来的（`robot_description/CMakeLists.txt`）：

```
urdf/mobile_manipulator.urdf.xacro   ← 手改这个（【量】647 行）
        │  xacro（构建期，add_custom_command）
        ▼
build/robot_description/mobile_manipulator.urdf     ← 展开产物（【量】669 行，别手改）
        │  install
        ▼
install/robot_description/share/robot_description/urdf/
        ├── mobile_manipulator.urdf          ← 展开后的纯 URDF
        └── mobile_manipulator.urdf.xacro    ← xacro 源也装出去（改形状的人该改它）
```

这么做的收益：**消费方运行期不依赖 xacro** —— mock、SDK、任何 `ros2` 工具拿到
`mobile_manipulator.urdf` 就能直接用（【量】文件头写着 `autogenerated by xacro`）。

xacro 的三个参数：

| 参数 | 默认 | 作用 |
|---|---|---|
| `hardware` | `mock` | `mock` = 假硬件（`colcon test` 走这条）；`gazebo` = `gz_ros2_control` |
| `ros2_control_params` | 空 | 控制器配置的**绝对路径**，由 launch 传 |
| `enable_camera` | `true` | 关掉深度相机省带宽（headless 下默认关） |

### 4.2 三条"发布/使用"路径

**① 参数 `robot_description`（字符串）**

`robot_state_publisher` 在**自己的参数**里拿着它。`ros2 param get` 能读到。
这是 launch 里最常见的给法：

```python
parameters=[{"robot_description": robot_description}]     # robot_description 是 XML 字符串
```

**② 话题 `/robot_description`（`std_msgs/String`，latched）**

`robot_state_publisher` 会把描述**同时发到话题上**。**实测 QoS【量】**：

```
Type: std_msgs/msg/String
Reliability: RELIABLE
Durability: TRANSIENT_LOCAL     ← latched：后到的订阅者也能拿到那一份
```

谁靠这条话题吃饭（都在各自 launch 的注释里记着）：

- **`controller_manager`（ros2_control）** —— 它从**话题**拿描述，**不是**参数。
  `ros2_control.launch.py` 里那条 `robot_state_publisher` 就是为它起的
  （少了它硬件永远不加载）。
- **`ros_gz_sim create`** —— `gazebo.launch.py` 里 `-topic robot_description`，
  在 Gazebo 里生成机器人的那一步也从话题拿。

**③ 直接读文件**

- `move_group.launch.py`：`MoveItConfigsBuilder(...).robot_description(file_path="urdf/mobile_manipulator.urdf")`
  —— 相对 `robot_description` 的 share 目录。
- **mock**：参数 `robot_description_file`（【量】`mock_robot_node.cpp:180`），走
  `jointSpecsFromFile()` 只取**关节的类型/限位/速度上限**。不喂它，mock 就不查限位
  （这正是 `robot_sdk_tests` 的 fixture 要喂它的原因）。
- RViz 的 RobotModel `Description Source: File`（§2.4 ①）。

### 4.3 谁用哪一部分

| 消费方 | 拿什么 | 通过哪条路 |
|---|---|---|
| `robot_state_publisher` | 整份 → 算 TF | 参数 → 再发话题 |
| `controller_manager` | `<ros2_control>` 块 | **话题** |
| Gazebo (`ros_gz_sim create`) | 整份 | **话题** |
| MoveIt (`move_group`) | URDF + SRDF + kinematics | **文件** |
| mock | 关节类型/限位/速度上限 | 文件（参数 `robot_description_file`） |
| RViz `RobotModel` | 几何（视觉/碰撞） | 参数 / 话题 / 文件，三选一 |
| Nav2 / costmap | **不用 URDF** | 用 `maps/room.pgm`（房间尺寸在 `worlds/sdk_demo.sdf` 和 mock 的常量里对齐） |

> URDF 是**机器人的属性**（能到哪、多快），所以"超限的目标该不该拒"归**执行侧**判，
> SDK 不查（它硬件无关）。这条纪律见 `robot_sdk_tests/README.md` 最后一节。

### 4.4 手边工具

```bash
# 手动展开（看某个 hardware 变体长什么样）
xacro src/robot_description/urdf/mobile_manipulator.urdf.xacro hardware:=gazebo > /tmp/r.urdf

# 校验 + 画连杆树
check_urdf /tmp/r.urdf
urdf_to_graphiz /tmp/r.urdf            # 生成 r.pdf

# 读回来
ros2 param get /robot_state_publisher robot_description
ros2 topic echo /robot_description --once
```

`check_urdf` 和 `urdf_to_graphiz` 来自 `liburdfdom-tools`，没装的话
`sudo apt install liburdfdom-tools`。

### 4.5 这个区域常见的坑

- **多行 XML 不能用 `-p` 传**（§2.4）——它是按行解析的。
- **改 URDF 时要一起改的地方**：房间尺寸有三处对齐（`worlds/sdk_demo.sdf` 的 x ±5 / y ±4、
  mock 的 `kRoomHalfWidth/kRoomHalfHeight`、`robot_sdk_nav2/maps/room.pgm`），改一处得三处一起改。
- **`shoulder_lift_joint` 的限位 ±1.8** 在 URDF 里，`probe_limits` 里也硬编码了一份
  （改 URDF 就得改它）——这是"两处各存一份"的已知代价。

---

## 5. 没有图形环境怎么办

你是 SSH 进来的，`DISPLAY` 是空的。三个选择：

**① Foxglove（推荐）—— 走端口转发，不碰 X11**

和"在 SSH 下看 Doxygen HTML"完全同一套流程：

```bash
sudo apt install ros-jazzy-foxglove-bridge
ros2 run foxglove_bridge foxglove_bridge          # 监听 8765
```

VSCode → **PORTS** 面板 → 转发 `8765` → 本地浏览器打开 `app.foxglove.dev`
→ Open connection → `ws://localhost:8765`。有 3D 面板，也有 Plot 面板（看位姿/电量随时间）。

**② 在 VM 自己的桌面上跑**

这台 VM 有活着的 X 会话（【量】`/tmp/.X11-unix/X0`、`X1`，属主 `dfg`）。如果能看到 VM 的
控制台窗口（VMware/VirtualBox 那个），直接在上面开个终端跑 `rviz2` 最快。

**③ X11 转发（`ssh -X`）**

RViz 走 X11 是 OGRE/GLX 过网络，通常很卡甚至起不来。局域网下能试，但不推荐。

**④ 不想开 GUI 的兜底**（纯 CLI）

```bash
ros2 run tf2_ros tf2_echo map base_footprint    # 实时打印位置/姿态
ros2 run tf2_tools view_frames                  # 生成 TF 树 PDF
ros2 topic hz /scan                             # 确认数据在动
```

---

## 6. 常见问题

**RViz 里什么都没有。**
先看 **Fixed Frame**（`map`）和 **TF display**：TF 里没有 `map` 就是数据没来。
再查 mock 是否在跑（`ros2 node list`）。最后查 QoS —— §2.3 坑一。

**`rviz2` 报 `cannot open display`。**
没有图形环境，见 §5。

**画面上有激光但位置不对 / 传感器帧在跳。**
§2.3 坑二：mock 和 URDF 的传感器帧打架了。只信一个来源。

**改了 `mock.rviz` 但 `rviz:=true` 没变化。**
`rviz` 目录是 install 出去的；`--symlink-install` 下软链会跟着源目录走，重新
`colcon build --packages-select robot_sdk_sim` 一下就同步了。
