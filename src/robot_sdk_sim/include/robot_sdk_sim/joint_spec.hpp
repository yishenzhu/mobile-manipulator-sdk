#pragma once

#include <map>
#include <string>
#include <vector>

#include "robot_sdk/result.hpp"

namespace robot_sdk::sim {

/// URDF 里一个关节"我们能用的那几样"。
///
/// 只取这些：不动几何（那是 TF / rviz 的事），也不管惯性（那是仿真的事）。
/// **不放在 robot_sdk_core 里** —— 这是"机器人自己的属性"，归执行侧（本模拟器 /
/// 真机上的控制器）读，SDK 只管发意图。两处各存一份迟早会漂移。
struct JointSpec {
  std::string name{};

  /// 关节能不能动。`fixed` 的不能。
  bool movable{false};

  /// 无限转的关节（URDF 的 `continuous`）。对它们，"目标 − 当前"走**最短的一边**。
  bool continuous{false};

  /// 上下限。**只在 `limited()` 为真时有意义**（否则是 0，别读）。
  double lower{0.0};
  double upper{0.0};

  /// 有限位吗？
  ///
  /// **不用单独记一个 bool** —— 它和上面两个字段是**恒等**的：urdfdom 强制要求
  /// revolute / prismatic **必须**带 `<limit>`（缺了直接解析失败，实测过），
  /// 而 continuous / fixed 根本没有上下限可言。所以"有限位"就等于"可动且不连续"。
  bool limited() const { return movable && !continuous; }

  /// 速度上限。URDF 没写就是 0 —— 调用方该退回自己的兜底值。
  double max_velocity{0.0};
};

/// 从 URDF 文件里挑出 `wanted` 点名的关节，**按名字索引**。
///
/// 一份 URDF 里有底盘、轮子、传感器…… 而"哪几个归 motion 管"是配置说了算，
/// URDF 说不出来。分工是：**配置点名，URDF 补充属性**。
///
/// 返回 map 而不是 vector：调用方**只按名字查**（"这个关节能不能转到 X"），
/// 没人关心顺序 —— 那就让容器表达这件事，别让每个调用点自己写一遍线性查找。
///
/// 读不到 / 解析不了 / 缺关节 → 返回空表，**不抛**：模拟器少一份描述也该能起来
/// （只是不查限位、不折角而已），那是它以前的行为。
std::map<std::string, JointSpec> jointSpecsFromFile(const std::string &path,
                                                    const std::vector<std::string> &wanted);

} // namespace robot_sdk::sim
