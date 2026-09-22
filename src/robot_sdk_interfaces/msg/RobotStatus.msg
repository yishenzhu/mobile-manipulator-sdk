# 机器人聚合健康状态与运行状态。
# 这是 SDK 对外暴露的唯一真相来源；底层可能由多个 topic/service 汇总而来。

std_msgs/Header header

# 高层运行状态。
uint8 STATE_UNKNOWN=0
uint8 STATE_IDLE=1
uint8 STATE_NAVIGATING=2
uint8 STATE_MOVING=3
uint8 STATE_CHARGING=4
uint8 STATE_ESTOP=5
uint8 STATE_ERROR=6

uint8 state

# 电源子系统。
float32 battery_percentage
bool is_charging
bool is_battery_low
float32 battery_voltage

# 故障上报。error_code == 0 表示健康。
int32 error_code
string error_message

# 诊断信息。
float64 uptime_seconds
bool is_estop_engaged
