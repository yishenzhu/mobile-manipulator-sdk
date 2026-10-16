// rayToBox 是**纯函数**（房间尺寸 + 射线 → 距离），所以能直接单测。
// 它承重的是"激光看到什么"和"前方挡不挡路"，以前完全没有测试：
// 改挂载点或房间尺寸时只会静默变数字。这里把解析值钉住。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "robot_sdk_sim/mock_world.hpp"

namespace robot_sdk::sim {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

TEST(RayToBox, HitsTheNearestWall) {
  // 房间 x ∈ [-5,5]、y ∈ [-4,4]。从原点沿 ±x / +y 打，命中距离就是那面墙的距离。
  EXPECT_DOUBLE_EQ(rayToBox(0.0, 0.0, 0.2, 1.0, 0.0, 0.0), 5.0);
  EXPECT_DOUBLE_EQ(rayToBox(0.0, 0.0, 0.2, -1.0, 0.0, 0.0), 5.0);
  EXPECT_DOUBLE_EQ(rayToBox(0.0, 0.0, 0.2, 0.0, 1.0, 0.0), 4.0);
}

TEST(RayToBox, DiagonalHitsTheNearerWall) {
  // 45° 斜射：x 方向要 5√2、y 方向只要 4√2 ⇒ 命中的是更近的那面（4√2）。
  const double s = std::sqrt(0.5);
  EXPECT_NEAR(rayToBox(0.0, 0.0, 1.0, s, s, 0.0), 4.0 * std::sqrt(2.0), 1e-9);
}

TEST(RayToBox, MountHeightDoesNotChangeHorizontalHits) {
  // 这是本测试最要紧的一条：挂载高度改由 URDF 提供后，数值变了会不会影响扫描？
  // 房间 z ∈ [0, 2.5]，水平射线只要在这个范围内，命中距离与高度无关。
  EXPECT_DOUBLE_EQ(rayToBox(0.0, 0.0, 0.16, 1.0, 0.0, 0.0),
                   rayToBox(0.0, 0.0, 0.20, 1.0, 0.0, 0.0));
}

TEST(RayToBox, MissesWhenOutsideTheSlab) {
  // 起点在房间外（z 高于天花板）：水平射线永远打不到 ⇒ inf（ROS 里表示"没有回波"）。
  EXPECT_TRUE(std::isinf(rayToBox(0.0, 0.0, 3.0, 1.0, 0.0, 0.0)));
  // 朝上打，天花板在天上：起点 0.2 ⇒ 距离 2.3。
  EXPECT_DOUBLE_EQ(rayToBox(0.0, 0.0, 0.2, 0.0, 0.0, 1.0), 2.3);
}

TEST(RayToBox, OutsideOriginReturnsEntryDistance) {
  // ⚠️ 起点在**房间外**时给的是「进入房间的距离」（t_enter），不是穿过房间后的出口。
  // 这条分支不常用，但抽取函数时最容易顺手写错（写成 t_exit 就静默变成另一个数）。
  EXPECT_DOUBLE_EQ(rayToBox(7.0, 0.0, 1.0, -1.0, 0.0, 0.0), 2.0);   // 从 x=+7 朝 -x ⇒ 在 x=+5 进入
  EXPECT_DOUBLE_EQ(rayToBox(0.0, 0.0, 3.0, 0.0, 0.0, -1.0), 0.5);   // 从天花板外朝下 ⇒ 在 z=2.5 进入
}

}  // namespace
}  // namespace robot_sdk::sim
