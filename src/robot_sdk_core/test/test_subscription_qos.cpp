#include <gtest/gtest.h>

#include <string>

#include "robot_sdk/config.hpp"

using namespace robot_sdk;

// 边界转换：字符串 → Policy。只认三个值，其余落成 Unknown（不抛），正确性交给使用方。
TEST(SubscriptionQosTest, ReliabilityFromStrings) {
  EXPECT_EQ(SubscriptionQos::reliabilityFrom("best_effort"),
            rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(SubscriptionQos::reliabilityFrom("reliable"),
            rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(SubscriptionQos::reliabilityFrom("system_default"),
            rclcpp::ReliabilityPolicy::SystemDefault);
  // 不支持的两个：best_available（实测等价于 best_effort 且有 discovery race）、unknown（哨兵）
  EXPECT_EQ(SubscriptionQos::reliabilityFrom("best_available"),
            rclcpp::ReliabilityPolicy::Unknown);
  EXPECT_EQ(SubscriptionQos::reliabilityFrom("unknown"), rclcpp::ReliabilityPolicy::Unknown);
  EXPECT_EQ(SubscriptionQos::reliabilityFrom("reliablee"), rclcpp::ReliabilityPolicy::Unknown);
  EXPECT_EQ(SubscriptionQos::reliabilityFrom(""), rclcpp::ReliabilityPolicy::Unknown);
  EXPECT_EQ(SubscriptionQos::reliabilityFrom("BEST_EFFORT"), rclcpp::ReliabilityPolicy::Unknown);
}

TEST(SubscriptionQosTest, ToQosCarriesDepthAndReliability) {
  SubscriptionQos sub;
  sub.depth = 10;
  sub.reliability = "best_effort";
  const auto qos = sub.toQos();
  EXPECT_EQ(qos.depth(), 10u);
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);  // 我们的订阅都是 volatile

  sub.reliability = "reliable";
  EXPECT_EQ(sub.toQos().reliability(), rclcpp::ReliabilityPolicy::Reliable);

  sub.depth = 0;  // depth <= 0 不该拼出深度 0 的非法 QoS
  EXPECT_EQ(sub.toQos().depth(), 1u);
  sub.depth = -3;
  EXPECT_EQ(sub.toQos().depth(), 1u);
}

/// ⚠️ 把文档里的订阅契约钉在代码里：谁改默认值这里会红，逼他去改文档并想清为什么。
TEST(SubscriptionQosTest, DefaultsAreTheDocumentedContract) {
  const QosConfig qos;
  EXPECT_EQ(qos.sensor.reliability, "best_effort");  // 传感器：订阅端必须更宽松
  EXPECT_EQ(qos.sensor.depth, 10);
  EXPECT_EQ(qos.joint_states.reliability, "best_effort");  // 高频流；深度不同故单列
  EXPECT_EQ(qos.joint_states.depth, 5);
  EXPECT_EQ(qos.status.reliability, "reliable");  // 低频、"最新一条才有意义"
  EXPECT_EQ(qos.status.depth, 1);
}
