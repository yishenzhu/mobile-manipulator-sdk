// Robot SDK 的最小示例：连上机器人，读状态和传感器。
//
//   # 终端 1：启动 mock 机器人
//   ros2 run robot_sdk_sim mock_robot_node
//   # 终端 2：跑本示例
//   ros2 run robot_sdk_examples hello_robot
//
// 全程不需要 Gazebo 或真机 —— 换成真机时，**这个文件一行都不用改**。
#include <chrono>
#include <cstdio>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "robot_sdk/robot.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // 一个 Robot = 四个能力 API（导航/运动/感知/状态）+ 它们需要的 ROS 2 管道。
  //
  // 执行器仍然是 **ROS 2 的**：SDK 内部自建了一个 MultiThreadedExecutor
  // （见 robot_sdk_core/src/runtime.cpp），`robot.spin()` 转的就是它。
  // 这个 std::thread 不是为了替代执行器 —— 只是因为 `spin()` 会**阻塞**，
  // 而主线程后面还要继续调阻塞式 API（waitForXxx / goTo / wait）。
  // （寄生模式 `Robot(node, config)` 下由客户自己 spin，连这个线程都不用。）
  robot_sdk::RobotConfig config{"hello_robot"};
  robot_sdk::Robot robot(config);
  std::thread spinner([&robot]() { robot.spin(); });

  std::printf("=== Robot SDK hello ===\n");

  // ── 状态：两种拿法，回答的是不同的问题 ──

  // ① 缓存快照：等来**这一条**，字段就从它上面读 ——
  //    别一边拿着"这一条"，一边又去调 API 读"此刻最新那一条"（那是两个来源）。
  if (auto status = robot.status().waitForStatus(5s); status.ok()) {
    std::printf("状态  : state=%u  电量=%.1f%%  充电中=%d\n", status.value()->state,
                status.value()->battery_percentage, status.value()->is_charging);
  } else {
    std::printf("状态  : %s\n", status.error().toString().c_str());
  }

  // API 上还有几个"就地判断"的便捷方法 —— 它们读的是**此刻最新**那条，
  // 未必和上面拿到的是同一条。要合成判断（比如"健康"是若干字段的综合）就用它：
  std::printf("健康  : %d\n", robot.status().isHealthy() ? 1 : 0);

  // ② 服务查询：一次请求/应答，"此刻确切的"。
  if (auto queried = robot.status().queryStatus(5s); queried.ok()) {
    std::printf("服务  : 已运行 %.1f 秒\n", queried.value().uptime_seconds);
  } else {
    std::printf("服务  : %s\n", queried.error().toString().c_str());
  }

  // ── 传感器：都是"最新一帧"语义，**不是消息队列** ——
  //    每秒 10 帧的雷达，你晚来一步就直接拿到最新那帧，不会排队等你消费。
  if (auto scan = robot.perception().waitForLaserScan(5s); scan.ok()) {
    // 注意：`nearestObstacleDistance` 是 API 上算出来的便捷量，读的是**最新一帧**，
    // 严格说不保证就是我们手上这一帧。这里图省事这么写；
    // 要严格对应，就自己在我们拿到的这一帧的 ranges 上找最小值。
    std::printf("激光  : %zu 条射线，正前方最近障碍 %.2f m\n",
                scan.value()->ranges.size(),
                robot.perception().nearestObstacleDistance(-0.02, 0.02));
  } else {
    std::printf("激光  : %s\n", scan.error().toString().c_str());
  }

  if (auto imu = robot.perception().waitForImu(5s); imu.ok()) {
    std::printf("IMU   : 角速度 z = %.3f rad/s\n",
                imu.value()->angular_velocity.z);
  }

  if (auto depth = robot.perception().waitForDepthImage(5s); depth.ok()) {
    std::printf("相机  : 深度图 %ux%u (%s)\n", depth.value()->width,
                depth.value()->height, depth.value()->encoding.c_str());
  }

  if (auto cloud = robot.perception().waitForPointCloud(5s); cloud.ok()) {
    std::printf("相机  : 点云 %ux%u\n", cloud.value()->width,
                cloud.value()->height);
  }

  // ── 位姿：从 TF 查，不是订阅一个"位姿话题" ──
  if (auto pose = robot.navigation().currentPose(2s); pose.ok()) {
    std::printf("位姿  : x=%.2f  y=%.2f  yaw=%.2f\n", pose.value().x,
                pose.value().y, pose.value().yaw);
  } else {
    std::printf("位姿  : %s\n", pose.error().toString().c_str());
  }

  robot.stop();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
