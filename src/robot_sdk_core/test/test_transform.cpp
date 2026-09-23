#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "robot_sdk/transform.hpp"
#include "robot_sdk/types.hpp"

using namespace robot_sdk;
using namespace std::chrono_literals;

namespace {

class TransformTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

/// 同帧变换是恒等变换 —— 这条路径完全不碰 TF，所以不需要任何 TF 数据就能测。
/// 也正因为不碰 TF，它是"最便宜的测试"：构造对象 + 一次调用。
TEST_F(TransformTest, TransformPoseToItsOwnFrameIsIdentity) {
  auto node = std::make_shared<rclcpp::Node>("tf_identity_test");
  TransformManager tf(node);

  geometry_msgs::msg::PoseStamped in;
  in.header.frame_id = "map";
  in.pose.position.x = 1.5;
  in.pose.position.y = -2.0;
  in.pose.position.z = 0.7;

  const auto out = tf.transformPose(in, "map");

  ASSERT_TRUE(out.ok()) << out.error().toString();
  EXPECT_EQ(out.value().header.frame_id, "map");
  EXPECT_DOUBLE_EQ(out.value().pose.position.x, 1.5);
  EXPECT_DOUBLE_EQ(out.value().pose.position.y, -2.0);
  EXPECT_DOUBLE_EQ(out.value().pose.position.z, 0.7);
}

/// 不存在的坐标系：tf2 会抛异常，我们的封装必须把它转成 Error，
/// 而不是让异常穿透出去（那是 SDK 的硬约束）。
TEST_F(TransformTest, MissingFrameBecomesUnavailableError) {
  auto node = std::make_shared<rclcpp::Node>("tf_missing_test");
  TransformManager tf(node);

  const auto res = tf.lookupTransform("map", "no_such_frame_at_all", 100ms);

  EXPECT_FALSE(res.ok());
  EXPECT_EQ(res.error().code, ErrorCode::kUnavailable);
  EXPECT_FALSE(res.error().message.empty());  // 必须带上 tf2 的原始原因

  EXPECT_FALSE(tf.canTransform("map", "no_such_frame_at_all"));
}

/// 正向路径：真发一个静态变换，验证能查出来。
/// 这是"查不到"之外的另一半 —— 光测失败路径不算覆盖。
TEST_F(TransformTest, LookupSucceedsWithPublishedStaticTransform) {
  auto node = std::make_shared<rclcpp::Node>("tf_static_test");
  auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node->now();
  t.header.frame_id = "tf_test_parent";
  t.child_frame_id = "tf_test_child";
  t.transform.translation.x = 3.0;
  t.transform.translation.y = 4.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.w = 1.0;
  broadcaster->sendTransform(t);

  TransformManager tf(node);

  // 静态变换是 latched 的，但 listener 线程收到它仍需要一点时间 ——
  // 轮询等待，最多 2 秒。避免用固定 sleep（那才是测试不稳定的根源）。
  bool ready = false;
  for (int i = 0; i < 100 && !ready; ++i) {
    ready = tf.canTransform("tf_test_parent", "tf_test_child");
    if (!ready) {
      std::this_thread::sleep_for(20ms);
    }
  }
  ASSERT_TRUE(ready) << "静态变换在 2 秒内未到达 TF buffer";

  const auto pose = tf.currentPose("tf_test_parent", "tf_test_child", 1s);
  ASSERT_TRUE(pose.ok()) << pose.error().toString();
  EXPECT_NEAR(pose.value().x, 3.0, 1e-9);
  EXPECT_NEAR(pose.value().y, 4.0, 1e-9);
  EXPECT_NEAR(pose.value().yaw, 0.0, 1e-9);
}

}  // namespace
