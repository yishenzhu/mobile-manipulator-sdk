#include <gtest/gtest.h>

#include "robot_sdk/detail/joint_wrap.hpp"

using namespace robot_sdk;

namespace {

constexpr double kPi = kTwoPi / 2.0;

}  // namespace

/// 当前 0.3、目标 7.0：绕一圈还多 0.417 —— 该走 0.417，不是 6.7。
TEST(JointWrapTest, TakesTheShortWayAround) {
  EXPECT_NEAR(0.4168, shortestAngleDelta(0.3, 7.0), 1e-4);
}

/// 反方向同理：7.0 → 0.3 该走 −0.417。
TEST(JointWrapTest, Symmetric) {
  EXPECT_NEAR(-0.4168, shortestAngleDelta(7.0, 0.3), 1e-4);
}

/// 不动就是不动（不能因为折角反而转起来）。
TEST(JointWrapTest, ZeroForSameAngle) {
  EXPECT_DOUBLE_EQ(0.0, shortestAngleDelta(1.234, 1.234));
}

/// 多绕几圈也塌回同一小段。
TEST(JointWrapTest, ManyTurnsCollapse) {
  EXPECT_NEAR(0.1, shortestAngleDelta(0.0, kTwoPi + 0.1), 1e-9);
  EXPECT_NEAR(-0.1, shortestAngleDelta(0.0, -kTwoPi - 0.1), 1e-9);
  EXPECT_NEAR(0.1, shortestAngleDelta(0.0, 10.0 * kTwoPi + 0.1), 1e-9);
}

/// 结果恒在 (−π, π] —— 恰好半圈时归到 +π（两边一样近，取哪边都行，但必须有个定论）。
TEST(JointWrapTest, AlwaysWithinHalfTurn) {
  EXPECT_NEAR(kPi, shortestAngleDelta(0.0, kPi), 1e-9);
  EXPECT_NEAR(kPi, shortestAngleDelta(0.0, -kPi), 1e-9);

  // 扫一遍：任何目标都不该超过半圈。
  for (double to = -20.0; to <= 20.0; to += 0.37) {
    const double delta = shortestAngleDelta(0.7, to);
    EXPECT_LE(delta, kPi + 1e-12) << "to=" << to;
    EXPECT_GT(delta, -kPi - 1e-12) << "to=" << to;
  }
}
