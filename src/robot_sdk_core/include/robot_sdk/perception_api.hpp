#pragma once

#include <chrono>
#include <limits>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "robot_sdk/config.hpp"
#include "robot_sdk/detail/latest_value_cache.hpp"
#include "robot_sdk/result.hpp"

namespace robot_sdk {

/// 机器人板载传感器的访问入口。
///
/// 传感器是**持续流式**的，所以这里暴露的是「最新样本」+「阻塞等待」，
/// 而不是消息队列。timeout 传 0（默认值）表示"用配置里的
/// perception.sensor_timeout_s"。
class PerceptionAPI {
 public:
  using Duration = std::chrono::duration<double>;

  using LaserScan = sensor_msgs::msg::LaserScan;
  using Imu = sensor_msgs::msg::Imu;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  // 读路径返回 ConstSharedPtr 而不是按值 —— 这是刻意的：
  //   - 读操作只增加一次引用计数，O(1)，与消息大小无关
  //   - 若按值返回，Image(≈600 KB) / PointCloud2(可达数 MB) 每次读都要深拷贝
  //   - const 让缓存发出去的数据谁都改不了 —— 编译期免疫一整类数据竞争
  //   - 这本来就是 ROS 2 订阅回调的原生类型，零转换成本
  using LaserScanPtr = LaserScan::ConstSharedPtr;
  using ImuPtr = Imu::ConstSharedPtr;
  using ImagePtr = Image::ConstSharedPtr;
  using CameraInfoPtr = CameraInfo::ConstSharedPtr;
  using PointCloud2Ptr = PointCloud2::ConstSharedPtr;

  /// 只接收自己需要的那一份配置（接口隔离）。
  ///
  /// 注意话题名用的是**相对名**（"scan" 这种），不自己拼命名空间前缀 ——
  /// rcl 会在创建订阅时按节点命名空间自动展开。想要不同的命名空间，
  /// 正确做法是把节点建在那个命名空间里，而不是逐名字覆盖。
  PerceptionAPI(const rclcpp::Node::SharedPtr& node,
                const rclcpp::CallbackGroup::SharedPtr& callback_group,
                const PerceptionConfig& config);

  std::optional<LaserScanPtr> laserScan() const { return laser_scan_.latest(); }
  std::optional<ImuPtr> imu() const { return imu_.latest(); }
  std::optional<ImagePtr> depthImage() const { return depth_image_.latest(); }
  std::optional<CameraInfoPtr> cameraInfo() const { return camera_info_.latest(); }
  std::optional<PointCloud2Ptr> pointCloud() const { return point_cloud_.latest(); }

  bool hasLaserScan() const { return laser_scan_.hasValue(); }
  bool hasImu() const { return imu_.hasValue(); }
  bool hasDepthImage() const { return depth_image_.hasValue(); }
  bool hasCameraInfo() const { return camera_info_.hasValue(); }
  bool hasPointCloud() const { return point_cloud_.hasValue(); }

  Result<LaserScanPtr> waitForLaserScan(Duration timeout = Duration{0.0});
  Result<ImuPtr> waitForImu(Duration timeout = Duration{0.0});
  Result<ImagePtr> waitForDepthImage(Duration timeout = Duration{0.0});
  Result<CameraInfoPtr> waitForCameraInfo(Duration timeout = Duration{0.0});
  Result<PointCloud2Ptr> waitForPointCloud(Duration timeout = Duration{0.0});

  /// 最新一帧激光在 [angle_min, angle_max] 窗口内的最近有效距离。
  /// 没有可用回波时返回无穷大，调用方可以直接和安全阈值比较。
  double nearestObstacleDistance(
      double angle_min = -std::numeric_limits<double>::infinity(),
      double angle_max = std::numeric_limits<double>::infinity()) const;

 private:
  Duration resolveTimeout(Duration timeout) const;

  template <typename MsgT>
  using Cache = LatestValueCache<typename MsgT::ConstSharedPtr>;

  rclcpp::Node::SharedPtr node_;
  PerceptionConfig config_;

  Cache<LaserScan> laser_scan_;
  Cache<Imu> imu_;
  Cache<Image> depth_image_;
  Cache<CameraInfo> camera_info_;
  Cache<PointCloud2> point_cloud_;

  rclcpp::Subscription<LaserScan>::SharedPtr laser_scan_subscription_;
  rclcpp::Subscription<Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<Image>::SharedPtr depth_image_subscription_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<PointCloud2>::SharedPtr point_cloud_subscription_;
};

}  // namespace robot_sdk
