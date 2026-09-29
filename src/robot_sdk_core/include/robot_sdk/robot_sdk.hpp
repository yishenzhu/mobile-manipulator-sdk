#pragma once

/// Umbrella header：一次拉进 Robot SDK 的全部公开 C++ API。
///     #include <robot_sdk/robot_sdk.hpp>
/// 不含 `detail/` 下的内部件（要算最近障碍距离请走 nearestObstacleDistance()）。

#include "robot_sdk/action_handle.hpp"
#include "robot_sdk/config.hpp"
#include "robot_sdk/error.hpp"
#include "robot_sdk/motion_api.hpp"
#include "robot_sdk/navigation_api.hpp"
#include "robot_sdk/perception_api.hpp"
#include "robot_sdk/result.hpp"
#include "robot_sdk/robot.hpp"
#include "robot_sdk/status_api.hpp"
#include "robot_sdk/transform.hpp"
#include "robot_sdk/types.hpp"
#include "robot_sdk/version.hpp"
