#pragma once

#include <cmath>

namespace robot_sdk {

/// 一圈的弧度值。**不引 M_PI** —— 它是 POSIX 扩展，标准 C++ 里不保证有。
constexpr double kTwoPi = 6.283185307179586476925286766559;

/// 连续关节（能无限往一个方向转的那种）该走的最短角差。
///
/// 对这类关节，`目标 - 当前` 和 `它 ± 2πk` 是**同一个物理姿态**，直接相减会绕远路：
/// 当前 0.3、目标 7.0 时，它会老老实实转 6.7 rad（≈384°），其实只差 0.417 rad（≈24°）。
/// **只对旋转关节用** —— 移动关节的单位是米，套上去毫无意义。
///
/// 值域是半开的 `(-π, π]`：恰好半圈时一律归到 `+π`，免得同一个角度有两个代表。
/// 纯函数：不碰节点、不碰时间、不碰全局状态，所以能零成本单元测试。
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
