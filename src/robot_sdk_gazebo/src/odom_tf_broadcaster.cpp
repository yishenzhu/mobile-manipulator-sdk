// 把 Gazebo 的 /odom 话题转成 odom → base 的 TF。
// 为什么要它：gz 的 DiffDrive 只发 /odom 话题、**不发 TF**；而 Nav2 要的是
// map → odom → base 这条完整的链（其中 map→odom 由 SLAM 或 AMCL 给）。
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace robot_sdk::gazebo_bridge {

class OdomTfBroadcaster : public rclcpp::Node {
 public:
  explicit OdomTfBroadcaster(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("odom_tf_broadcaster", options) {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // 里程计是高频流，用 SensorDataQoS —— 和发布端一致，不一致会静默收不到。
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onOdometry(*msg); });
  }

 private:
  void onOdometry(const nav_msgs::msg::Odometry& odom) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    // frame_id 留空时退回参数 —— 有的驱动不带它，那会让 TF 直接报错。
    transform.header.frame_id =
        odom.header.frame_id.empty() ? odom_frame_ : odom.header.frame_id;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    broadcaster_->sendTransform(transform);
  }

  std::string odom_topic_;
  std::string odom_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

}  // namespace robot_sdk::gazebo_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_sdk::gazebo_bridge::OdomTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
