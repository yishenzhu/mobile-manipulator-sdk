#pragma once

#include <string>

// 注意：rclcpp_action::ResultCode 定义在 client_goal_handle.hpp，
// **不在** types.hpp 里（types.hpp 只有 GoalUUID / GoalStatus / GoalInfo）。
//
// 写错这个 include 的后果很隐蔽：头文件不再自包含，只有当别的头文件
// 恰好先引入了 client_goal_handle.hpp 时才能编译过。一旦有人第一个
// include 本文件，就会看到 "'ResultCode' is not a member of 'rclcpp_action'"
// 这种莫名其妙的报错。
#include <rclcpp_action/client_goal_handle.hpp>

namespace robot_sdk {

/// 把 action 的终态码转成人类可读字符串（用于日志和错误信息）。
inline std::string resultCodeToString(rclcpp_action::ResultCode code) {
  // 与项目里的 ErrorCode::toString 同一策略：穷尽列出所有取值、不写 default，
  // 让 -Wswitch 在 rclcpp_action 将来新增终态码时提醒我们。
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    case rclcpp_action::ResultCode::UNKNOWN:
      return "UNKNOWN";
  }
  return "UNKNOWN";  // 兜底：枚举变量理论上可能被塞非法值
}

}  // namespace robot_sdk
