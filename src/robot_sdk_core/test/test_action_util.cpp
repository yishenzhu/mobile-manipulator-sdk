// 本文件**故意只 include detail/action_util.hpp**（外加 gtest）—— 这是它的第二个
// 作用：充当"头文件自包含性测试"。原来那个头文件的 include 写错了，却因为没人单独
// 编译它一直没被发现；单独 include 一个头文件能立刻暴露这类问题。
#include <gtest/gtest.h>

#include "robot_sdk/detail/action_util.hpp"

TEST(ActionUtilTest, CoversEveryResultCode) {
  EXPECT_EQ(robot_sdk::resultCodeToString(rclcpp_action::ResultCode::SUCCEEDED), "SUCCEEDED");
  EXPECT_EQ(robot_sdk::resultCodeToString(rclcpp_action::ResultCode::ABORTED), "ABORTED");
  EXPECT_EQ(robot_sdk::resultCodeToString(rclcpp_action::ResultCode::CANCELED), "CANCELED");
  EXPECT_EQ(robot_sdk::resultCodeToString(rclcpp_action::ResultCode::UNKNOWN), "UNKNOWN");
}
