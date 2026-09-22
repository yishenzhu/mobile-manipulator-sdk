#include <gtest/gtest.h>

#include "robot_sdk/config.hpp"

using namespace robot_sdk;

TEST(ConfigTest, DefaultsAreUsable) {
  const RobotConfig cfg;
  EXPECT_EQ(cfg.navigation.action_name, "navigate_to_pose");
  EXPECT_EQ(cfg.navigation.global_frame, "map");
  EXPECT_EQ(cfg.navigation.base_frame, "base_footprint");
  EXPECT_EQ(cfg.motion.action_name, "move_to_joint_positions");
  EXPECT_EQ(cfg.status.status_topic, "robot_status");
  EXPECT_EQ(cfg.perception.laser_scan_topic, "scan");
  EXPECT_GT(cfg.navigation.default_timeout_s, 0.0);
  EXPECT_GT(cfg.perception.sensor_timeout_s, 0.0);
}

TEST(ConfigTest, NestedStructsAreDefaultConstructed) {
  const RobotConfig cfg;
  EXPECT_TRUE(cfg.motion.joint_names.empty());    // 空 vector，不是未初始化的垃圾
  EXPECT_TRUE(cfg.motion.named_targets.empty());  // 空 map
  EXPECT_TRUE(cfg.robot_namespace.empty());       // 空 string
  EXPECT_FALSE(cfg.use_sim_time);
  EXPECT_EQ(cfg.motion.gripper_closed_position, 0.0);
}
