#pragma once

#include <cmath>

namespace robot_sdk {

/// 一圈的弧度值。**不引 M_PI** —— 它是 POSIX 扩展，标准 C++ 里不保证有。
constexpr double kTwoPi = 6.283185307179586476925286766559;

/// 连续关节该走的最短角差（值域半开的 `(-π, π]`，恰好半圈归到 `+π`）。
/// 直接相减会绕远路：当前 0.3、目标 7.0 实际只差 0.417 rad，不减 2π 会转 6.7 rad。
/// **只对旋转关节用**（移动关节单位是米）。纯函数，可零成本单测。
inline double shortestAngleDelta(double from, double to) {
  double delta = std::fmod(to - from, kTwoPi);
  if (delta > kTwoPi / 2.0) {
    delta -= kTwoPi;
  } else if (delta <= -kTwoPi / 2.0) {
    delta += kTwoPi;
  }
  return delta;
}

} // namespace robot_sdk
