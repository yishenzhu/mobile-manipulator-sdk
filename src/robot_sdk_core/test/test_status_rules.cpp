// `state_bits` 与 `error_code` 的**唯一解释处**（robot_sdk::allowsMotion / allowsNavigation）。
//
// 位掩码的核心收益在这里被钉住：**臂在动时底盘照样能动**（反之亦然）——
// 旧的单值 state（IDLE/NAVIGATING/MOVING/…）表达不了这种组合，只能一刀切。
// 也用纯函数覆盖 ESTOP / ERROR 两位 —— mock 不产生它们，但判据必须对。
#include <gtest/gtest.h>

#include "robot_sdk/status_api.hpp"

namespace robot_sdk {
namespace {

using Bits = robot_sdk_interfaces::msg::RobotStatus;

RobotStatus statusWith(uint32_t bits, int32_t error_code = 0) {
  RobotStatus status;
  status.state_bits = bits;
  status.error_code = error_code;
  return status;
}

TEST(StatusRules, EachChannelOnlyBlockedByItself) {
  // 这是本次改动存在的理由：两个通道互不阻塞。
  EXPECT_TRUE(motionAllowed(statusWith(Bits::STATE_BASE_MOVING)));
  EXPECT_TRUE(navigationAllowed(statusWith(Bits::STATE_ARM_MOVING)));
  // 两个都在动 ⇒ 两个都不许。
  EXPECT_FALSE(motionAllowed(statusWith(Bits::STATE_BASE_MOVING | Bits::STATE_ARM_MOVING)));
  EXPECT_FALSE(navigationAllowed(statusWith(Bits::STATE_BASE_MOVING | Bits::STATE_ARM_MOVING)));
}

TEST(StatusRules, IdleChargingAndLowBatteryStillAllowMotion) {
  EXPECT_TRUE(motionAllowed(statusWith(0)));
  EXPECT_TRUE(navigationAllowed(statusWith(0)));
  // 充电不是禁止动作的理由（执行侧本来就只按"忙"拦）。
  EXPECT_TRUE(motionAllowed(statusWith(Bits::STATE_CHARGING)));
  EXPECT_TRUE(navigationAllowed(statusWith(Bits::STATE_CHARGING)));
  EXPECT_TRUE(motionAllowed(statusWith(Bits::STATE_BATTERY_LOW)));
}

TEST(StatusRules, EstopBlocksEveryChannel) {
  const uint32_t estop = Bits::STATE_ESTOP;
  EXPECT_FALSE(motionAllowed(statusWith(estop)));
  EXPECT_FALSE(navigationAllowed(statusWith(estop)));
  // 急停与"正在动"可以并存（急停后可能还在滑停）—— 判据不受影响。
  EXPECT_FALSE(motionAllowed(statusWith(estop | Bits::STATE_ARM_MOVING)));
}

TEST(StatusRules, ErrorCodeBlocksEveryChannel) {
  // "有没有故障"就看 error_code（不再单设 ERROR 状态位）。
  EXPECT_FALSE(motionAllowed(statusWith(0, /*error_code=*/1)));
  EXPECT_FALSE(navigationAllowed(statusWith(0, /*error_code=*/1)));
}

}  // namespace
}  // namespace robot_sdk
