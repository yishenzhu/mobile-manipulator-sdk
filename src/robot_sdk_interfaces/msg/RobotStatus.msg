# 机器人状态位与遥测。
#
# ⚠️ state_bits 是**位掩码**，不是枚举：同一时刻可以有多位成立（例如"底盘在动 + 充电中"）。
# 旧版是单值枚举（IDLE/NAVIGATING/MOVING/…），而"谁在动"本来就是**两个独立通道**的事实 ——
# 单值装不下，同时动时会丢信息。
#
# 判断"能不能发某个动作"**不要各自做位运算**：走 SDK 的 allows_motion() / allows_navigation()
# （判据一处定义，含 fail-closed 与 error_code）。

std_msgs/Header header

# ── 运动：两个独立通道，可以同时成立 ──
uint32 STATE_BASE_MOVING   = 1    # bit0 底盘在动（导航动作 **或** cmd_vel 直接控速）
uint32 STATE_ARM_MOVING    = 2    # bit1 机械臂在动
# ── 电源 / 安全 ──
uint32 STATE_CHARGING      = 4    # bit2 充电中
uint32 STATE_ESTOP         = 8    # bit3 急停
uint32 STATE_BATTERY_LOW   = 16   # bit4 低电（阈值是机器人的属性，由生产者判好给位）
# bit5..31 预留

uint32 state_bits

# 电源遥测：位只回答"是否"，具体值在这里。
float32 battery_percentage
float32 battery_voltage

# 故障上报。error_code == 0 表示健康 —— "有没有故障"就看它，不再另设状态位。
int32 error_code
string error_message

# 诊断信息。
float64 uptime_seconds
