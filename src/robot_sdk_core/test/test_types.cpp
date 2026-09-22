#include <gtest/gtest.h>

#include <cmath>

#include "robot_sdk/types.hpp"

using namespace robot_sdk;

namespace {
constexpr double kPi = M_PI;
}

TEST(TypesTest, NormalizeAngleWrapsIntoMinusPiPi) {
  EXPECT_NEAR(0.0, normalizeAngle(0.0), 1e-9);
  EXPECT_NEAR(-kPi, normalizeAngle(kPi), 1e-9);
  EXPECT_NEAR(-kPi + 0.1, normalizeAngle(kPi + 0.1), 1e-6);
  EXPECT_NEAR(0.5, normalizeAngle(2.0 * kPi + 0.5), 1e-9);
  EXPECT_NEAR(-0.5, normalizeAngle(-2.0 * kPi - 0.5), 1e-9);
}

TEST(TypesTest, AngleDifferenceTakesShortestPath) {
  EXPECT_NEAR(-0.2, angleDifference(0.1, 0.3), 1e-9);
  EXPECT_NEAR(0.2, angleDifference(0.3, 0.1), 1e-9);
  EXPECT_NEAR(0.2, angleDifference(-kPi + 0.1, kPi - 0.1), 1e-6);
}

TEST(TypesTest, DistanceIsPlanar) {
  EXPECT_DOUBLE_EQ(5.0,
                   distance(Pose2D{0.0, 0.0, 0.0}, Pose2D{3.0, 4.0, 1.23}));
}

TEST(TypesTest, PoseRoundTripsThroughMessage) {
  const Pose2D original{1.5, -2.0, 0.7};
  const auto round_tripped = toPose2D(toPoseMsg(original));
  EXPECT_NEAR(original.x, round_tripped.x, 1e-9);
  EXPECT_NEAR(original.y, round_tripped.y, 1e-9);
  EXPECT_NEAR(original.yaw, round_tripped.yaw, 1e-9);
}

TEST(TypesTest, YawRoundTripsThroughQuaternion) {
  for (const double yaw : {-3.0, -1.5, 0.0, 0.25, 1.5, 3.0}) {
    EXPECT_NEAR(yaw, yawFromQuaternion(quaternionFromYaw(yaw)), 1e-9);
  }
}