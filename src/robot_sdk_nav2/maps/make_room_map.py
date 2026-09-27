#!/usr/bin/env python3
"""照 mock 机器人的房间，生成一张静态地图（room.pgm + room.yaml）。

**为什么要生成、而不是拿现成的**：地图是"世界的一个快照"。拿一张和 mock 房间
对不上的图（比如 Nav2 自带的 tb3_sandbox），代价地图上就会长出一堆 mock 里
根本不存在的墙，规划器直接罢工。真部署里这张图是 SLAM 跑出来的，这里我们直接画。

房间的尺寸来自 mock 的 mock_robot_node.cpp：
    kRoomHalfWidth  = 5.0   → x ∈ [-5, 5]
    kRoomHalfHeight = 4.0   → y ∈ [-4, 4]

    python3 make_room_map.py
"""

import os

HALF_W = 5.0          # 与 mock 的 kRoomHalfWidth 一致
HALF_H = 4.0          # 与 mock 的 kRoomHalfHeight 一致
RESOLUTION = 0.05     # 与两份 nav2_params.yaml 里的 resolution 一致
MARGIN = 0.5          # 四周留点余量，免得墙正好压在图像边缘上

FREE = 254            # PGM 里 0 = 全黑（占用）、255 = 全白（空闲）
OCCUPIED = 0


def main():
    origin_x = -HALF_W - MARGIN
    origin_y = -HALF_H - MARGIN
    width = int(round((2 * HALF_W + 2 * MARGIN) / RESOLUTION))
    height = int(round((2 * HALF_H + 2 * MARGIN) / RESOLUTION))

    # 先按"像素 = 世界坐标"填：房间**外面**是墙，里面是空地。
    # 墙在 x = ±HALF_W、y = ±HALF_H 这两个平面上，所以从那个像素起就是占用。
    grid = [[FREE] * width for _ in range(height)]
    for py in range(height):
        # py 从**下往上**数（地图坐标系是左下为原点）
        world_y = origin_y + (py + 0.5) * RESOLUTION
        for px in range(width):
            world_x = origin_x + (px + 0.5) * RESOLUTION
            if abs(world_x) >= HALF_W or abs(world_y) >= HALF_H:
                grid[py][px] = OCCUPIED

    out_dir = os.path.dirname(os.path.abspath(__file__))
    pgm_path = os.path.join(out_dir, "room.pgm")
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n")
        f.write(f"# mock robot room: x+-{HALF_W} y+-{HALF_H}\n".encode())
        f.write(f"{width} {height}\n255\n".encode())
        # ★ PGM 的行是**从上到下**，而地图原点在**左下** —— 所以这里必须倒着写。
        #   忘了翻，整张图会上下颠倒，而且**不报错**。
        for py in reversed(range(height)):
            f.write(bytes(grid[py]))

    yaml_path = os.path.join(out_dir, "room.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"image: room.pgm\n")
        f.write(f"resolution: {RESOLUTION:.6f}\n")
        f.write(f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.000000]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")

    print(f"  {pgm_path}")
    print(f"  {width} x {height} 像素 @{RESOLUTION} m/px  →  "
          f"{width * RESOLUTION:.1f} x {height * RESOLUTION:.1f} m")
    print(f"  原点 ({origin_x:.1f}, {origin_y:.1f})；空地 x ∈ [{-HALF_W}, {HALF_W}]、"
          f"y ∈ [{-HALF_H}, {HALF_H}]")
    print(f"  {yaml_path}")


if __name__ == "__main__":
    main()
