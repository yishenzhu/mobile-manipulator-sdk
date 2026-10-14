# 契约：什么是公开 API，怎么改才不破坏客户

一句话：**接口只增不删**（语义化版本）。机械部分由 `scripts/check_contract.py` 在 CI 里守，语义部分靠这份文档 ——
脚本只能抓"删了/改名了/换类型了"，抓不到"C++ 方法改名""错误码含义变了"。

## 公开契约有哪几面

| 面 | 具体是什么 | 谁在守 |
|---|---|---|
| ROS 接口 | `robot_sdk_interfaces` 的 4 个文件（1 msg + 3 action）的字段与类型 = **线协议** | 脚本 |
| C++ 公开头 | `robot_sdk/*.hpp`（13 个，含 `action_handle.hpp`）；`detail/` 里的**不算** | 人 |
| Python 面 | `robot_sdk` 的类/方法名、`ErrorCode` 枚举值、`RobotSdkError` | 人 |
| MCP 工具面 | 11 个工具名 + 参数名 + `{ok, code, message, data}` 信封 | 人 |
| 配置面 | ROS 参数名（`qos.sensor.reliability` 等）与环境变量名（`ROBOT_SDK_*`） | 人 + 脚本（版本） |

`detail/` 下的东西（`action_handle.hpp` 之外的 `action_slot.hpp`、`latest_value_cache.hpp`、`joint_wrap.hpp`、
`scan_utils.hpp`、`action_util.hpp`、`runtime.hpp`）**不是**契约，可以随便改、随便删。

## 什么算破坏（要升 major）

- **接口**：删字段、字段改名、改字段类型、在已有字段**中间**插新字段（CDR 按顺序序列化，后面全移位）、删掉整块。
- **语义**：错误码含义变了（例如 `Timeout` 改成表示"忙"）、QoS 默认值**收紧**（可能让订阅静默收不到数据）、
  以前合法的调用被新校验拒掉 —— **收紧校验就是破坏**，这也是"忙就拒、不排队"这种设计要留意的地方。
- **面**：工具改名或去掉参数、Python 方法改名、`ErrorCode` 删值。

## 什么不算（升 minor / patch）

- 在某个段**末尾**追加字段（消费者要按"未知值当未知处理"写）、加新工具、加新错误码、加带默认值的参数、
  **放宽**校验（以前拒的现在收）、性能提升、文档。
- 新增一个 `.msg` / `.action` **文件**属于"加块" ⇒ 允许（检查器按块比，不会误报）。

## 怎么改（配方）

1. **加字段**：加在该段**末尾** + 给合理默认 + 跑 `--update` 更新基线 ⇒ review 里就是"追加某行"，一眼看完。
2. **废弃**：先保留并标注（注释 + 文档），**别删**；到下一个 major 才删。删除/改名永远等 major。
3. **改语义**：改不动就别改 —— 新增字段/枚举值/工具来表达新语义，旧含义保持不动。
4. **版本**：整个工作区**一个版本号**，不是每包一个。理由：Python 扩展是**编译期**连 C++ 核心与接口的，
   版本错配根本装不起来 ⇒ 分开版本没有意义，只会制造"该升哪个包"的伪问题。

## 机械保证（`scripts/check_contract.py`）

- **版本一致**：`VERSION` 是唯一声明，14 个文件 / 15 处（11 个 `package.xml` + 2 个 `setup.py` +
  `version.hpp` 的数字与字符串两项）必须与它一致 —— 手写这么多处，最容易悄悄漂；
  其中 `version.hpp` 最危险（编进公开头，漂了就直接发给客户）。
- **只增不删**：4 个接口文件摊成 `docs/contract-baseline.txt` 的声明清单，**按块比**，基线必须是当前的**前缀**。
  末尾追加 ⇒ 打印并放行；中间插 / 删 / 改名 / 换类型 ⇒ 退出码 1。
- **阴性验证过**（避免"只会说通过"的检查）：末尾追加字段 ⇒ 放行；把 `avoid_obstacles` 改名 ⇒
  拦下并指出"第 3 项起变了"；`VERSION` 改成 0.2.0 ⇒ 15 处全部报不一致。
- **抓不到的**：C++/Python 面的改名、语义变化、QoS 默认值变化 ⇒ 靠 review + 本文档。别把脚本当护身符。

## 发版（当前 **0 个 tag**）

```bash
# 1) VERSION 与 15 处声明改到新版本   2) 更新 CHANGELOG.md
python3 scripts/check_contract.py      # 3) 必须绿
git tag -a "v$(cat VERSION)" -m "v$(cat VERSION)"
```
