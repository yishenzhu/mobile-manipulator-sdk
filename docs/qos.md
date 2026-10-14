# QoS 契约：我们订阅什么，以及"静默收不到"是怎么发生的

> 这份文档回答三个问题：**SDK 的订阅用什么 QoS？不一致会怎样？怎么改？**
> 相关文档：`file-map.md`（谁在哪）、`sdk-call-flow.md`（调用流）。

纪律照旧：【量】= 本机跑过、有数字；【推】= 从框架语义推导。

## 0. 一句话

**传感器与关节流按 BEST_EFFORT 订阅，状态按 RELIABLE 订阅**，两者都可配。QoS **不散在业务配置里**：
统一放在 `RobotConfig::qos`（`QosConfig`），由 `Robot` 装配时按**角色**分发给各 API（见 §1、§4）。
选这个默认值不是偏好，而是**兼容规则**逼出来的：QoS 不匹配时**数据一条都到不了**（你的代码只会看到 TIMEOUT）——
rclcpp 会在节点日志里打一条 WARN（实测，见 §3），但日志容易被忽略，且它不指出该改哪个配置项。

## 1. 当前契约：我们订阅什么

| 订阅 | 主题 | `reliability` | depth | 为什么 |
|---|---|---|---|---|
| 激光 / IMU / 深度图 / 点云 / camera_info | `scan` `imu` `camera/depth/…` | **best_effort** | `qos.sensor.depth`（默认 10） | 【推】传感器惯例是 BEST_EFFORT；订阅端**必须**与之相等或更宽松 |
| 机器人状态 | `robot_status` | **reliable** | 1 | 低频、"最新一条才有意义"，状态不该丢 |
| 关节状态 | `joint_states` | **best_effort** | 5 | 高频流，与 `ros2_control` / Gazebo 的发布端一致 |
| 状态服务客户端 | `get_robot_status` | `ServicesQoS` | — | 服务语义 |

**durability 一律 volatile，不暴露**：上述主题都不是 latched 的，而 volatile 订阅能接受任何发布者
⇒ 没有可配的必要（RxO 细节见 §2.2）。真出现 TRANSIENT_LOCAL 的需求再加字段（那时也才知道默认值该给什么）。

## 2. RxO：哪几个 QoS 维度会参与"匹配"

【推】DDS 把发布端与订阅端配对时，会逐个比较一组叫 **RxO（Requested/Offered）** 的策略。
规则一句话：**订阅端"要求"的强度 ≤ 发布端"提供"的强度**（offered ≥ requested）。不满足 ⇒
两端**不匹配** ⇒ 数据**一条都不过去**。这是本仓库最该记住的一条 DDS 语义。
（"静默"的准确含义见 §3：**你的代码只会看到 TIMEOUT**；rclcpp 会在节点日志里打一条 WARN，但那容易被忽略。）

参与匹配的**只有 4 个维度**（其余见 §2.5）：

| 维度 | 强度的阶梯（左弱右强） | 订阅端"安全侧" |
|---|---|---|
| `reliability` | `BEST_EFFORT` < `RELIABLE` | **BEST_EFFORT** |
| `durability` | `VOLATILE` < `TRANSIENT_LOCAL` | **VOLATILE** |
| `deadline` | 无限（不承诺） < 有限值（越短越严） | **无限** |
| `liveliness` | `AUTOMATIC` < `MANUAL_BY_NODE` < `MANUAL_BY_TOPIC` | **AUTOMATIC** |

所谓"订阅端安全侧" = 该维度上**最不挑剔**的取值 —— 取它则**与任何发布端都兼容**。

⚠️ 阶梯之外还有个 `system_default`（交给中间件的默认），**可以配**（见 §4）。

**不支持的**是 `best_available` 与 `unknown`，两者都落成 `Unknown`（不抛）。理由：
`best_available` —— rmw 自己警告策略在**建订阅时**定死、之后不会因新发现的端点改变
（"non-deterministic behavior can occur due to races with discovery"），且【量】**本环境（Jazzy + FastDDS）
实测它等价于 `best_effort`**（连"等发现完成后再建订阅"也没兑现"能可靠就可靠"）；
`unknown` —— rmw 定义它是"尚未设置"的哨兵，不是可请求的策略。

⚠️ 但**不是"枚举里有就都能配"**：唯一被排除的是 `unknown` —— rmw 对它的定义是
"Reliability policy **has not yet been set**"，它是未初始化的**哨兵**，不是可请求的策略。
判据因此是"**是不是一个可请求的策略**"，而不是"在不在枚举里"（`rmw/types.h` 的注释就写着这条依据）。

### 2.1 `reliability`【量】（本仓库实测过，见 §3）

| 发布端提供 | 订阅端要求 | 兼容？ |
|---|---|---|
| BEST_EFFORT | BEST_EFFORT | ✅ |
| RELIABLE | BEST_EFFORT | ✅（订阅端自愿放弃重传保证） |
| **BEST_EFFORT** | **RELIABLE** | ❌ **静默无数据** |
| RELIABLE | RELIABLE | ✅ |

### 2.2 `durability`【推】

| 发布端提供 | 订阅端要求 | 兼容？ | 说明 |
|---|---|---|---|
| VOLATILE | VOLATILE | ✅ | 迟到的订阅者不补历史 |
| **TRANSIENT_LOCAL** | **VOLATILE** | ✅ | 发布端有缓存，订阅端说"我不要" |
| TRANSIENT_LOCAL | TRANSIENT_LOCAL | ✅ | 后连上者**立刻收到**缓存（= ROS 1 的 latch） |
| **VOLATILE** | **TRANSIENT_LOCAL** | ❌ **静默无数据** | 订阅端索要发布端根本不保留的历史 |

⇒ 有人为了"更稳妥"把它调成 `TRANSIENT_LOCAL`，面对默认的 `VOLATILE` 发布端会**永久收不到任何数据**。
这是本仓库**不暴露**这个维度的直接原因。

### 2.3 `deadline`【推】

比的是"谁的期限更严"：**发布端承诺的间隔 O 必须 ≤ 订阅端要求的间隔 R**。

| 发布端提供 | 订阅端要求 | 兼容？ |
|---|---|---|
| 无限（不承诺） | 无限（不要求） | ✅ 默认 |
| 100 ms | 无限 | ✅（发布端做得比要求更好） |
| 100 ms | 200 ms | ✅ |
| **无限** | **100 ms** | ❌ **静默无数据**（发布端不肯承诺期限） |

⇒ 订阅端**默认无限**才是安全侧；一旦设成有限值，碰上"不承诺期限"的发布端就静默失效。

### 2.4 `liveliness`（+ lease duration）【推】

kind 要满足 offered ≥ requested；选了 MANUAL 系时**还要比较 lease duration** —— 都对得上才匹配。

| 发布端提供 | 订阅端要求 | 兼容？ |
|---|---|---|
| AUTOMATIC | AUTOMATIC | ✅ 默认 |
| AUTOMATIC | MANUAL_BY_TOPIC | ❌ **静默无数据** |
| MANUAL_BY_TOPIC | MANUAL_BY_TOPIC | ✅（lease duration 也要匹配得上） |

### 2.5 不参与匹配的维度（只影响本地，安全）

`history`（KEEP_LAST / KEEP_ALL）、`depth`（队列长度）、`lifespan`（样本寿命）**不参与 RxO**：
它们只影响本地缓存与投递行为，**不会把话题弄哑**。所以暴露它们是安全的（`depth` 我们已暴露）。

⚠️ 反过来看：**RxO 那 4 个维度里，任何一个被客户调得"更严"，都会让话题静默失效。**
这就是"RxO 旋钮不能随手给"的理由，也是 §6 那个自检建议比"多暴露几个字段"更值的原因。

## 3. 实测：同一个发布端，两种订阅【量】

`/scan` 由一个 **BEST_EFFORT** 发布端发出，SDK 分别用两种配置订阅。
下表"订阅端 QoS"读的是 **真实 DDS QoS**（`ros2 topic info -v /scan`），不是配置字段：

| `qos.sensor.reliability` | 订阅端 QoS（实测） | `wait_for_laser_scan` |
|---|---|---|
| `best_effort`（默认） | `Reliability: BEST_EFFORT` | **收到 360 条射线** ✓ |
| `reliable` | `Reliability: RELIABLE` | **`TIMEOUT: no laser scan received`** ✗ |

三个结论：① 配置**真的作用到 DDS 层**（不是只改了个内存字段）；② **不兼容 ⇒ 数据到不了**（代码只会看到 TIMEOUT）；
③ **rclcpp 其实会打一条 WARN**（实测原话，上一版本文档漏了它 —— 因为我把输出按 `^\[` 过滤了）：

```
[WARN] [qos_probe]: New publisher discovered on topic '/scan', offering incompatible QoS.
       No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

所以"静默"是**对代码而言**没有错误信号；**对日志并非无声** —— 但日志容易被忽略，而且它不指出该改哪个配置项（§6）。

## 4. 怎么改

```python
cfg = robot_sdk.RobotConfig()
cfg.qos.sensor.reliability = "reliable"      # 5 个感知话题
cfg.qos.status.reliability = "best_effort"   # 真机状态发布端是 BEST_EFFORT 时就该这样改
cfg.qos.sensor.depth = 10                    # KEEP_LAST 深度（即原来的 perception.queue_depth）
```

`reliability` 在配置里是**字符串**：ROS 参数 / launch / 环境变量都只能给字符串，配置保持同一种形态最省事。
**支持的取值**：`best_effort` | `reliable` | `system_default`（不含 `best_available` / `unknown`，理由见 §2）。
字符串→枚举的转换是 `SubscriptionQos::reliabilityFrom()`（`toQos()` 用它）；不认识的值落成 `Unknown`，不抛。
⚠️ 代价：写错不会当场拦下 —— 【量】实测它一路走到 `rcl` 才报错，且消息是 `invalid allocator`（看不出是 QoS）。

另注：QoS 目前只能**按角色**配，做不到"按单个话题"配 —— 话题→QoS 的映射只能用代码 / `map` 表达，
而 **ROS 参数表达不了 map**（同 `MotionConfig::named_targets`）。

这条契约由 `test/test_subscription_qos.cpp` 钉住（含"默认值就是本文档的契约"那一条 —— 谁改默认值它就会红）。

从 launch 覆盖（配置字段与 ROS 参数同名）：

```
-p qos.sensor.reliability:=reliable -p qos.status.reliability:=best_effort
```

⚠️ **写错的值会当场抛**（构造期），不会静默退回默认值，理由有两条：

1. 写错 QoS 的后果本来就够隐蔽（静默收不到数据），配置错误必须第一时间拦下；
2. 构造期**没有 Result 通道** —— SDK 的规则是"**运行期**接口从不抛异常，**构造期**以 `SdkException`
   表达配置错误"（见 `result.hpp` 的类型注释），绑定的语言边界会把它统一翻成 Python 的
   `RobotSdkError(INVALID_ARGUMENT)`。所以这条 throw 落在既有的构造期豁免里，不是新开的口子。

这条契约由 `test/test_qos_util.cpp` 钉住（含"默认值就是本文档的契约"那一条 —— 谁改默认值它就会红）。

## 5. 复现 §3

```bash
# 终端 1：一个 BEST_EFFORT 的 /scan 发布端
python3 be_scan_pub.py

# 终端 2：分别用两种 reliability 订阅（脚本会打印结果 + 真实端点 QoS）
python3 check_qos_compat.py best_effort   # → 收到 360 条射线
python3 check_qos_compat.py reliable      # → TIMEOUT
```

⚠️ 脚本收尾必须 `robot.stop()` **再** `spinner.join()`：只 stop 不 join 的话，主线程退出时
`spin()` 还在跑、析构会和它抢执行器（实测：`terminate called without an active exception` + core dump）。

## 6. 已实现：把"静默"变成"大声"

先说一条**实测更正**：rclcpp **已经**在订阅端发现不兼容发布端时打 WARN 了（§3 那条原话）——
所以"静默"只是**对代码**而言（以前一律是 `TIMEOUT`）。要补的不是"再查一遍"，而是**把已有信息接到 SDK 的错误面上**。

**做法**：`wait_*` 超时的那一刻（**只在超时路径上查，成功路径零开销**）调 `Node::get_publishers_info_by_topic()`
+ `rclcpp::qos_check_compatible()`，把原因写进错误。实现只有一处：
`detail/subscription_wait.hpp::waitForCache()`，感知 5 个等待 + 状态 2 个等待共用它。

三条实测输出（原始打印，复现命令见 §5）：

| 场景 | `RobotSdkError` 的内容 |
|---|---|
| 有发布端但 QoS 不兼容 | `UNAVAILABLE: topic '/scan' has a publisher (qc_be_scan_pub) but its QoS is incompatible: ERROR: Best effort publisher and reliable subscription; [publisher reliability=best_effort, this subscription wants reliable]` |
| 话题上压根没有发布端 | `UNAVAILABLE: topic '/scan' has no publisher at all` |
| 有发布端、QoS 也兼容，只是还没数据 | `TIMEOUT: no laser scan received within timeout`（**措辞与改动前一字不差**） |

三个决定（都有理由，不是随手写的）：

- **超时那一刻去查，而不是接 QoS 事件**：查出来的是**真实策略**（上面那句 `Best effort publisher and reliable
  subscription` 是 rcl 给的原文），事件只给一个计数；而且顺手把"压根没有发布端"也区分出来了 ——
  这两种原因对应的动作完全不同（一个改 QoS、一个去查发布端为什么没起）。
- **原因明确时用 `Unavailable`，不再报 `Timeout`**：同样是"没数据"，责任方不同就该给不同代码，
  调用方（例如 agent）才能自己决定怎么办。**这是行为变化**（这些场景以前都是 `TIMEOUT`），已记入 CHANGELOG。
- **消息用英文**：SDK 的公开错误消息本来就是英文，中文只出现在文档里。

⚠️ 一处必须说明的写法：`qos_check_compatible()` 与 `get_publishers_info_by_topic()` 在 rclcpp 里**是会抛的**。
"运行期不抛异常"的约定不变 —— 这里在**边界**上接住并降级成"没查出来"（继续报 `Timeout`）：
诊断失败不该把一次 `Result` 变成异常。这是"接第三方会抛的 API"，不是"拿异常做控制流"。

还没做：① 把它固化成回归测试（目前只有手工验证的这三条）；② 我们自己当 publisher 时的同类诊断。
