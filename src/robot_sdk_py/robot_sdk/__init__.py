"""Robot SDK 的 Python 前端。

⚠️ 实现只有**一份**：全在 C++ 那份核心里（编译成扩展模块 `_robot_sdk`），这里只把名字导出来。
   所以 Python 和 C++ 的行为、错误语义完全一致 —— 不存在"两套实现慢慢漂移"这回事。

⚠️ 失败一律以 `RobotSdkError` 抛出：C++ 侧刻意不用异常（实时性 + 跨语言安全），
   而**语言边界正是最该抛的地方**。成功则把值投影成 dict。
"""

from ._robot_sdk import *  # noqa: F401,F403
from ._robot_sdk import (  # noqa: F401
    ErrorCode,
    JointMotionHandle,
    MotionAPI,
    MotionConfig,
    MotionGoal,
    NavigationAPI,
    NavigationConfig,
    NavigationGoal,
    NavigationHandle,
    PerceptionAPI,
    PerceptionConfig,
    Pose2D,
    PoseGoal,
    PoseMotionHandle,
    Robot,
    RobotConfig,
    RobotSdkError,
    StatusAPI,
    StatusConfig,
)
