cmake_minimum_required(VERSION 3.8)
project(robot_sdk_sim)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(robot_sdk_interfaces REQUIRED)
find_package(robot_sdk_core REQUIRED)

# 这是个可执行文件（不是库）：模拟机器人是个"程序"。
add_executable(mock_robot_node
    src/mock_robot_node.cpp
    src/mock_robot_main.cpp
)
# 注意这里**不用** $<BUILD_INTERFACE:...>：这个路径是 PRIVATE 的
# （不往外传播），而且这个可执行文件不导出 —— 所以"构建树 vs 安装树"的分歧
# 根本不存在，直接写路径即可。
# （那句生成器表达式只在路径要传播给别人时才有意义，见 robot_sdk_core 的写法。）
target_include_directories(mock_robot_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(mock_robot_node
    rclcpp
    geometry_msgs
    sensor_msgs
    tf2_ros
    robot_sdk_interfaces
    robot_sdk_core
)

# 假的下游：说 FollowJointTrajectory 的关节轨迹控制器替身，用来验运动链路。
add_executable(joint_trajectory_mock src/joint_trajectory_mock.cpp)
target_include_directories(joint_trajectory_mock PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(joint_trajectory_mock
    rclcpp
    rclcpp_action
    sensor_msgs
    control_msgs
    trajectory_msgs
    robot_sdk_core
)

# 可执行文件装到 lib/<包名>/ 下（ROS 2 的约定，ros2 run 就是从这里找）。
install(TARGETS mock_robot_node
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS joint_trajectory_mock
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp geometry_msgs sensor_msgs tf2_ros robot_sdk_interfaces robot_sdk_core)

ament_package()
