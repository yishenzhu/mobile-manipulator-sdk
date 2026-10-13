# 证据：一个句柄的引用环（LSan / TSan 原始报告）

本目录是 [`../action-handle-lifetime.md`](../action-handle-lifetime.md) 的**原始证据**。

**一句话结论**：修复**前** LSan 报 **22 处分配 / 1624 字节**泄漏（**16 条全是 `Indirect`**，无一条 `Direct` —— 环的指纹）；修复**后 0**。TSan 下四个示例 **0 条数据竞争**。

| 文件 | 是什么 | 大小 |
|---|---|---|
| `asan-leak-before.log` | 修复**前**的 LSan 报告（裁剪版，见下） | 11 KB |
| `asan-leak-after.log` | 修复**后** —— 只有示例的正常输出、**零** sanitizer 行（全文） | 770 B |
| `tsan-demos.log` | 四个示例在 TSan 下的输出（全文），每份的 `ThreadSanitizer` 行数 = 0 | 2 KB |
| `tsan-aslr-fatal.log` | Ubuntu 24.04 上 TSan 起不来的原始报错（→ 对策 `setarch -R`） | 342 B |
| `tsan-positive-control.cpp` | TSan **阳性对照**的源码（故意造数据竞争） | 914 B |
| `tsan-positive-control.log` | 它的 TSan 报告 —— 证明 TSan 在这个环境里**真的在工作** | 2 KB |

---

## 怎么产生的

**A/B 的做法**（关键：修复只有一行，所以不需要动 git）

把 `src/robot_sdk_core/include/robot_sdk/action_handle.hpp` 里的
`goal_future_ = {};` 临时注释掉（"修复前"），跑完再恢复（"修复后"）。
改回去后用 `git diff` 确认与提交版本逐字节一致。

**LSan（只看客户端侧，mock 不需要插桩）**

```bash
colcon build --symlink-install --packages-select robot_sdk_examples \
  --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -g" \
               -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address
install/robot_sdk_examples/lib/robot_sdk_examples/demo_navigation   # 对着 mock 跑
```

**TSan**

```bash
colcon build --symlink-install --packages-select robot_sdk_core robot_sdk_examples \
  --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=thread -g" -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread
setarch $(uname -m) -R install/robot_sdk_examples/lib/robot_sdk_examples/demo_navigation
```

> ⚠️ 两个坑：① **Ubuntu 24.04 的 ASLR** 会让 TSan 报 `FATAL: unexpected memory mapping`（见 `tsan-aslr-fatal.log`）—— 用 `setarch -R` 关掉该进程的 ASLR，**不改系统参数**；这个现象是**概率性**的（小程序有时不撞）。② **CMake 会缓存 sanitizer 参数**，不带参数重建清不掉 —— 必须 `rm -rf build/<pkg>`。

---

## 裁剪了什么（诚实说明）

- 原始 `asan_nofix.log`：**95,766 字节 / 305 行** → 这里的 `asan-leak-before.log`：**11 KB / 134 行**。
- **数字一字未改**：16 条的条数、字节数、`SUMMARY: AddressSanitizer: 1624 byte(s) leaked in 22 allocation(s).` 全部照抄。
- **只去掉**：asan/libc 内部帧、`BuildId`、超长模板名；每条泄漏保留**头部 + 分配点/我方/ROS 帧**（至多 6 帧），共省略 170 行帧。
- 其余五个文件**全文照抄，未裁剪**。

## 没有保留的

原始完整日志（95 KB 的 `asan_nofix.log` 等）在**会话临时目录**里，会随会话清理。需要原件时按
[`../action-handle-lifetime.md`](../action-handle-lifetime.md) §9 重跑一遍即可（约 1 分钟）。
