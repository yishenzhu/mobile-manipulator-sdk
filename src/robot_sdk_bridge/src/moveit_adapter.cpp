// 把 SDK 的笛卡尔动作翻译成 MoveIt 的 MoveGroup。
// 这是"**意图 → 规划约束**"那一层：客户给一个位姿，规划器要一组约束。逆解、碰撞检查、
// 规划全是 MoveIt 的事 —— 本节点只翻译和转发，没有本地状态要推进，不需要定时器。
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <rcl_action/rcl_action.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robot_sdk_interfaces/action/move_to_pose.hpp>

#include "robot_sdk/detail/action_slot.hpp"

namespace robot_sdk::bridge {

class MoveItAdapter : public rclcpp::Node {
 public:
  using SdkAction = robot_sdk_interfaces::action::MoveToPose;
  using SdkGoalHandle = rclcpp_action::ServerGoalHandle<SdkAction>;
  using MoveGroupAction = moveit_msgs::action::MoveGroup;
  using MoveGroupGoalHandle = rclcpp_action::ClientGoalHandle<MoveGroupAction>;

  explicit MoveItAdapter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : rclcpp::Node("moveit_adapter", options) {
    sdk_action_name_ = declare_parameter<std::string>("sdk_action_name", "sdk/move_to_pose");
    // MoveIt 自己的动作服务端叫 /move_action（move_group 节点提供的）。
    move_group_action_name_ =
        declare_parameter<std::string>("move_group_action_name", "/move_action");
    planning_group_ = declare_parameter<std::string>("planning_group", "");
    end_effector_link_ = declare_parameter<std::string>("end_effector_link", "");
    position_tolerance_ = declare_parameter<double>("position_tolerance", 0.01);
    orientation_tolerance_ = declare_parameter<double>("orientation_tolerance", 0.05);
    planning_attempts_ = declare_parameter<int>("planning_attempts", 3);
    planning_time_s_ = declare_parameter<double>("planning_time_s", 5.0);

    server_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 反馈里的"末端现在在哪"要自己查 TF —— MoveGroup 的反馈只有一个状态字符串，
    // 没有位姿。查不到就不编假位姿（见 currentEndEffectorPose）。
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    client_ = rclcpp_action::create_client<MoveGroupAction>(this, move_group_action_name_,
                                                            client_group_);
    server_ = rclcpp_action::create_server<SdkAction>(
        this, sdk_action_name_,
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const SdkAction::Goal> goal) {
          return handleGoal(uuid, std::move(goal));
        },
        [this](const std::shared_ptr<SdkGoalHandle> handle) { return handleCancel(handle); },
        [this](const std::shared_ptr<SdkGoalHandle> handle) { handleAccepted(handle); },
        rcl_action_server_get_default_options(), server_group_);
  }

 private:
  // ── 服务端回调 ──

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                         std::shared_ptr<const SdkAction::Goal> goal) {
    // 能查的只有"配置对不对"这一层：规划组和末端连杆是**部署相关**的，没有它们
    // 这条动作没法翻译 —— 这属于本节点自己的配置错误，不是机器人的属性。
    if (planning_group_.empty() || end_effector_link_.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "拒绝笛卡尔目标：planning_group / end_effector_link 没配（本节点参数）");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->target_pose.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "拒绝笛卡尔目标：target_pose 没有 frame_id");
      return rclcpp_action::GoalResponse::REJECT;
    }

    bool busy = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      busy = slot_.busy();
      if (!busy) {
        slot_.reserve(); // 占位：下游句柄要等回应回调才到
        // 位姿记下来：反馈和结果里的误差都要它，而 MoveGroup 的回调带不回来。
        target_pose_ = goal->target_pose;
      }
    }
    if (busy) {
      RCLCPP_WARN(get_logger(), "拒绝笛卡尔目标：上一个还没结束");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<SdkGoalHandle> sdk_handle) {
    if (!sdk_handle->is_active()) { // 问框架在锁外
      return rclcpp_action::CancelResponse::REJECT;
    }

    std::shared_ptr<MoveGroupGoalHandle> downstream;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slot_.cancel_requested = true;
      downstream = downstream_handle_;
    }
    // 下游句柄已经到手就立刻转发；还没到的话，等目标回应回调里补发。
    if (downstream) {
      client_->async_cancel_goal(downstream);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<SdkGoalHandle> sdk_handle) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slot_.begin(sdk_handle);
    }

    auto goal = std::make_shared<MoveGroupAction::Goal>(toMoveGroupGoal(*sdk_handle->get_goal()));

    rclcpp_action::Client<MoveGroupAction>::SendGoalOptions options;
    options.goal_response_callback = [this](const std::shared_ptr<MoveGroupGoalHandle> handle) {
      onGoalResponse(handle);
    };
    options.feedback_callback = [this](std::shared_ptr<MoveGroupGoalHandle>,
                                       const std::shared_ptr<const MoveGroupAction::Feedback> f) {
      onFeedback(f);
    };
    options.result_callback = [this](const MoveGroupGoalHandle::WrappedResult& wrapped) {
      onResult(wrapped);
    };

    client_->async_send_goal(*goal, options);
  }

  // ── 翻译：位姿 → 约束 ──

  /// 一个 PoseStamped 拆成两组约束：位置是**区域**（半径 = 容差的球），姿态是"朝向 ± 容差"。
  /// ⚠️ `any_orientation` 为真时**不填**姿态约束（MoveIt 语义：那一项空着 = 不管它）——
  ///    别填单位四元数，那意思是"朝基准方向"，是个真约束。
  moveit_msgs::msg::Constraints toConstraints(const geometry_msgs::msg::PoseStamped& target,
                                              bool any_orientation) const {
    moveit_msgs::msg::Constraints out;
    out.name = "sdk_pose_goal";

    moveit_msgs::msg::PositionConstraint position;
    position.header = target.header;
    position.link_name = end_effector_link_;
    position.constraint_region.primitives.resize(1);
    // ⚠️ `primitives.resize(1)` 只保证那个 primitive 存在，**不保证它内部的 dimensions
    //    有长度**。少下面这一句，再往下标写就是往空容器里写 ⇒ **段错误**（实测栽过：
    //    gdb 的栈顶就是 toConstraints）。
    auto& primitive = position.constraint_region.primitives[0];
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions.resize(3, 0.0);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = position_tolerance_;
    position.constraint_region.primitive_poses.resize(1);
    position.constraint_region.primitive_poses[0] = target.pose;
    position.weight = 1.0;
    out.position_constraints.push_back(position);

    if (!any_orientation) {
      moveit_msgs::msg::OrientationConstraint orientation;
      orientation.header = target.header;
      orientation.link_name = end_effector_link_;
      orientation.orientation = target.pose.orientation;
      orientation.absolute_x_axis_tolerance = orientation_tolerance_;
      orientation.absolute_y_axis_tolerance = orientation_tolerance_;
      orientation.absolute_z_axis_tolerance = orientation_tolerance_;
      orientation.parameterization =
          moveit_msgs::msg::OrientationConstraint::XYZ_EULER_ANGLES;
      orientation.weight = 1.0;
      out.orientation_constraints.push_back(orientation);
    }
    return out;
  }

  moveit_msgs::action::MoveGroup::Goal toMoveGroupGoal(const SdkAction::Goal& sdk) const {
    moveit_msgs::action::MoveGroup::Goal out;
    auto& request = out.request;
    request.group_name = planning_group_;
    request.goal_constraints.push_back(toConstraints(sdk.target_pose, sdk.any_orientation));
    request.num_planning_attempts = planning_attempts_;
    request.allowed_planning_time = planning_time_s_;

    // 客户的 max_velocity 是**速度**（m/s）⇒ 走 `max_cartesian_speed`（单位就是 m/s，
    // `<= 0` = 不修改轨迹，与 SDK 的"0 = 不指定"一致）。不能用 `max_velocity_scaling_factor`
    // ——那是**比例**，硬套会把 0.5 m/s 当成"半速"。⚠️ 要配 LimitMaxCartesianLinkSpeed 才生效。
    if (sdk.max_velocity > 0.0F) {
      request.cartesian_speed_limited_link = end_effector_link_;
      request.max_cartesian_speed = sdk.max_velocity;
    }

    // plan_only=false：要它规划**并执行**。客户发的是"到那儿"，不是"给我看看怎么走"。
    out.planning_options.plan_only = false;
    out.planning_options.planning_scene_diff.is_diff = true;
    return out;
  }

  // ── 下游客户端的三个回调 ──

  void onGoalResponse(const std::shared_ptr<MoveGroupGoalHandle> downstream) {
    std::shared_ptr<SdkGoalHandle> sdk_handle;
    bool cancel_late = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sdk_handle = slot_.handle;
      if (downstream) {
        downstream_handle_ = downstream;
        cancel_late = slot_.cancel_requested;
      }
    }

    if (!downstream) {
      if (sdk_handle) {
        auto result = std::make_shared<SdkAction::Result>();
        result->success = false;
        result->message = "MoveIt 拒绝了目标";
        sdk_handle->abort(result);
      }
      std::lock_guard<std::mutex> lock(mutex_);
      slot_.finish();
      return;
    }
    if (cancel_late) {
      client_->async_cancel_goal(downstream);
    }
  }

  /// MoveGroup 的反馈只有一个内部状态字符串（没有位姿、没有百分比），
  /// 所以完成度只能给"规划中 = 0，其余按 MoveIt 的状态分档"。末端位姿自己查 TF。
  void onFeedback(const std::shared_ptr<const MoveGroupAction::Feedback> feedback) {
    std::shared_ptr<SdkGoalHandle> sdk_handle;
    geometry_msgs::msg::PoseStamped last_goal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sdk_handle = slot_.handle;
      last_goal = target_pose_;
    }
    if (!sdk_handle || !sdk_handle->is_active()) {
      return;
    }

    auto out = std::make_shared<SdkAction::Feedback>();
    out->current_pose = currentEndEffectorPose(last_goal.header);
    // MoveIt 的 state 取值是 PLANNING / MONITOR / LOOK / IDLE 这类**阶段名**，
    // 没有连续进度可言 —— 这里只区分"还在规划"和"已经在动"。
    out->completion_percentage = feedback->state == "PLANNING" ? 0.0F : 50.0F;
    sdk_handle->publish_feedback(out);
  }

  /// 末端现在在哪。查不到就**只给 frame_id**（把位姿留成全零）——
  /// 编一个假位姿比空着更糟：客户没法分辨"它真在那儿"和"我们不知道"。
  geometry_msgs::msg::PoseStamped currentEndEffectorPose(
      const std_msgs::msg::Header& header) const {
    geometry_msgs::msg::PoseStamped out;
    out.header = header;
    try {
      const auto transform =
          tf_buffer_->lookupTransform(header.frame_id, end_effector_link_, tf2::TimePointZero);
      out.header = transform.header;
      out.pose.position.x = transform.transform.translation.x;
      out.pose.position.y = transform.transform.translation.y;
      out.pose.position.z = transform.transform.translation.z;
      out.pose.orientation = transform.transform.rotation;
    } catch (const tf2::TransformException&) {
      // 留空 —— 见上面的说明。
    }
    return out;
  }

  void onResult(const MoveGroupGoalHandle::WrappedResult& wrapped) {
    std::shared_ptr<SdkGoalHandle> sdk_handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sdk_handle = slot_.handle;
      slot_.finish();
      downstream_handle_.reset();
    }
    if (!sdk_handle) {
      return;
    }

    auto result = std::make_shared<SdkAction::Result>();
    if (wrapped.result) {
      result->success =
          wrapped.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      if (!result->success) {
        result->message = "MoveIt 规划/执行失败（error_code=" +
                          std::to_string(wrapped.result->error_code.val) + "）";
      }
      // ⚠️ 规划成功**不等于**走得到。SDK 的 final_position_error 说的是"实际离目标多远"，
      //    那要等执行完从 TF 上量 —— 不能拿"规划出来的终点"冒充（那是期望值，不是实测值）。
      //    这里查一次，查不到就留 0 并把话说清。
      geometry_msgs::msg::PoseStamped goal_pose;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_pose = target_pose_;
      }
      const auto actual = currentEndEffectorPose(goal_pose.header);
      if (!actual.header.frame_id.empty() && actual.pose.orientation.w != 0.0) {
        const auto& a = actual.pose.position;
        const auto& b = goal_pose.pose.position;
        result->final_position_error =
            std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                      (a.z - b.z) * (a.z - b.z));
        result->final_orientation_error = quaternionAngle(
            actual.pose.orientation, goal_pose.pose.orientation);
      } else if (result->message.empty()) {
        result->message = "到达了，但拿不到末端位姿（TF 上没有 " + end_effector_link_ + "）";
      }
    }

    // 终态**由下游决定**，本节点不自己判（和 motion_adapter 同一条规矩）。
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      result->success = false;
      if (result->message.empty()) {
        result->message = "运动被取消";
      }
      sdk_handle->canceled(result);
    } else if (wrapped.code == rclcpp_action::ResultCode::ABORTED && result->success) {
      result->success = false;
      if (result->message.empty()) {
        result->message = "运动被中止";
      }
      sdk_handle->abort(result);
    } else if (result->success) {
      sdk_handle->succeed(result);
    } else {
      sdk_handle->abort(result);
    }
  }

  /// 两个四元数之间的夹角（弧度）。用 |dot| 是为了让 q 和 -q 等价 —— 它们表示同一朝向。
  static double quaternionAngle(const geometry_msgs::msg::Quaternion& a,
                                const geometry_msgs::msg::Quaternion& b) {
    const double dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    return 2.0 * std::acos(std::min(1.0, std::abs(dot)));
  }

  std::string sdk_action_name_;
  std::string move_group_action_name_;
  std::string planning_group_;
  std::string end_effector_link_;
  double position_tolerance_ = 0.01;
  double orientation_tolerance_ = 0.05;
  int planning_attempts_ = 3;
  double planning_time_s_ = 5.0;

  rclcpp::CallbackGroup::SharedPtr server_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp_action::Server<SdkAction>::SharedPtr server_;
  rclcpp_action::Client<MoveGroupAction>::SharedPtr client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// 保护下面这些成员。回调分落在两条可重入车道上，所以它们真的会并发。
  std::mutex mutex_;
  ActionSlot<SdkAction> slot_;
  std::shared_ptr<MoveGroupGoalHandle> downstream_handle_;
  /// 本目标的位姿 —— 反馈和结果里的误差都要它（MoveGroup 的回调里带不回来）。
  geometry_msgs::msg::PoseStamped target_pose_;
};

} // namespace robot_sdk::bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<robot_sdk::bridge::MoveItAdapter>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
