#pragma once

#include <map>
#include <string>
#include <vector>

#include "robot_sdk/result.hpp"

namespace robot_sdk::sim {

/// URDF 里一个关节"我们能用的那几样"。只取这些：不动几何（TF/rviz 的事），
/// 也不管惯性（仿真的事）。**不放 robot_sdk_core** —— 这是机器人的属性，归执行侧读。
struct JointSpec {
  std::string name{};

  /// 关节能不能动。`fixed` 的不能。
  bool movable{false};

  /// 无限转的关节（URDF 的 `continuous`）。对它们，"目标 − 当前"走**最短的一边**。
  bool continuous{false};

  /// 上下限。**只在 `limited()` 为真时有意义**（否则是 0，别读）。
  double lower{0.0};
  double upper{0.0};

  /// 有限位吗？**不单独记 bool** —— 它与上面两个字段恒等：urdfdom 强制 revolute /
  /// prismatic 必须带 `<limit>`，而 continuous / fixed 没有上下限可言。
  bool limited() const { return movable && !continuous; }

  /// 速度上限。URDF 没写就是 0 —— 调用方该退回自己的兜底值。
  double max_velocity{0.0};
};

/// 从 URDF 文件里挑出 `wanted` 点名的关节，**按名字索引**（调用方只按名字查，
/// 不关心顺序 ⇒ 用 map 而不是 vector）。分工是"配置点名，URDF 补充属性"。
/// 读不到 / 解析不了 / 缺关节 ⇒ 返回空表、不抛：模拟器少一份描述也该能起来。
std::map<std::string, JointSpec> jointSpecsFromFile(const std::string &path,
                                                    const std::vector<std::string> &wanted);

} // namespace robot_sdk::sim
