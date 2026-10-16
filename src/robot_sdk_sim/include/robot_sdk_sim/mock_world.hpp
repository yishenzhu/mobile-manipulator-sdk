#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace robot_sdk::sim {

/// 模拟机器人的"世界"：一个长方体房间，**六面就是它全部的障碍物**。
/// 扫描和"前方有没有挡路的"都拿它算 —— 没有别的障碍物。
constexpr double kRoomHalfWidth = 5.0;  // x ∈ [-5, 5]
constexpr double kRoomHalfHeight = 4.0; // y ∈ [-4, 4]
// 房间是有高度的：地面 z=0、天花板 z=2.5。
// 2D 激光打不到它们（射线是水平的），但深度相机能 —— 所以相机需要三维的房间。
constexpr double kRoomFloorZ = 0.0;
constexpr double kRoomCeilingZ = 2.5;

/// 从 (ox,oy,oz) 沿**单位**方向射出，打在长方体房间上的距离；打不到返回 inf。
/// 标准 slab 法：三对平行板各给一个参数区间，求交。**激光和相机共用它** ——
/// 激光的 dz ≡ 0 等价于只看四面墙，共用一份房间模型就不会改了一个忘另一个。
inline double rayToBox(double ox, double oy, double oz, double dx, double dy, double dz) {
  const double origin[3] = {ox, oy, oz};
  const double dir[3] = {dx, dy, dz};
  const double lo[3] = {-kRoomHalfWidth, -kRoomHalfHeight, kRoomFloorZ};
  const double hi[3] = {kRoomHalfWidth, kRoomHalfHeight, kRoomCeilingZ};

  double t_enter = -std::numeric_limits<double>::infinity();
  double t_exit = std::numeric_limits<double>::infinity();

  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(dir[axis]) < 1e-12) {
      // 平行于这对板：不在两板之间就永远打不到。
      if (origin[axis] < lo[axis] || origin[axis] > hi[axis]) {
        return std::numeric_limits<double>::infinity();
      }
      continue;
    }
    double t1 = (lo[axis] - origin[axis]) / dir[axis];
    double t2 = (hi[axis] - origin[axis]) / dir[axis];
    if (t1 > t2) {
      std::swap(t1, t2); // 方向为负时近/远会颠倒
    }
    t_enter = std::max(t_enter, t1);
    t_exit = std::min(t_exit, t2);
  }

  // 区间空了（或整个盒子在身后）= 这条射线打不到墙。
  if (t_enter > t_exit || t_exit < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  // ⚠️ 关键：**相机在房间内部**，所以 t_enter 是负的，真正的墙是**出口** t_exit。
  //    把 t_enter 初始成 0 是错的 —— max(0, 负数) 会吃掉负号，于是永远返回 0
  //    （一整张深度图全是 0，而且不报错）。
  return t_enter > 0.0 ? t_enter : t_exit;
}

}  // namespace robot_sdk::sim
