#!/usr/bin/env python3
"""最小 Python 示例：连上机器人，读状态和传感器。全程不需要 Gazebo 或真机。

    ros2 run robot_sdk_sim mock_robot_node      # 终端 1
    python3 hello_robot.py                      # 终端 2

⚠️ 换成真机时这个文件**一行都不用改** —— 它只用了 SDK 的公开接口。
"""

import threading

import robot_sdk


def main() -> None:
    robot = robot_sdk.Robot(robot_sdk.RobotConfig(node_name="py_hello_robot"))

    # ⚠️ spin() 会阻塞，但它**放开了 GIL** ⇒ 后台线程能转，主线程还能继续调阻塞式 API。
    #    反过来（不放 GIL）会死锁：阻塞调用等的那个回调，正是这个线程该处理的。
    spinner = threading.Thread(target=robot.spin, daemon=True)
    spinner.start()

    print("=== Robot SDK hello (Python) ===")

    try:
        status = robot.status().wait_for_status(5.0)
        print(f"状态  : state={status['state']}  电量={status['battery_percentage']:.1f}%  "
              f"健康={robot.status().is_healthy()}")
    except robot_sdk.RobotSdkError as error:
        print(f"状态  : {error}")

    try:
        scan = robot.perception().wait_for_laser_scan(5.0)
        print(f"激光  : {scan['beam_count']} 条射线，正前方最近障碍 "
              f"{robot.perception().nearest_obstacle_distance(-0.02, 0.02):.2f} m")
        imu = robot.perception().wait_for_imu(5.0)
        print(f"IMU   : 角速度 = {imu['angular_velocity']}")
        depth = robot.perception().wait_for_depth_image(5.0)
        print(f"相机  : 深度图 {depth['width']}x{depth['height']} ({depth['encoding']})")
        cloud = robot.perception().wait_for_point_cloud(5.0)
        print(f"相机  : 点云 {cloud['width']}x{cloud['height']}")
    except robot_sdk.RobotSdkError as error:
        print(f"传感器: {error}")

    try:
        queried = robot.status().query_status(5.0)
        print(f"服务  : 已运行 {queried['uptime_seconds']:.1f} 秒")
    except robot_sdk.RobotSdkError as error:
        print(f"服务  : {error}")

    try:
        pose = robot.navigation().current_pose(2.0)
        print(f"位姿  : x={pose['x']:.2f}  y={pose['y']:.2f}  yaw={pose['yaw']:.2f}")
    except robot_sdk.RobotSdkError as error:
        print(f"位姿  : {error}")

    robot.stop()
    spinner.join(timeout=2.0)


if __name__ == "__main__":
    main()
