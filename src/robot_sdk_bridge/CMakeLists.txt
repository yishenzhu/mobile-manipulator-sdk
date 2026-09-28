cmake_minimum_required(VERSION 3.8)
project(robot_sdk_bridge)

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
find_package(nav2_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_sdk_interfaces REQUIRED)
find_package(robot_sdk_core REQUIRED)

add_executable(nav2_adapter src/nav2_adapter.cpp)
ament_target_dependencies(nav2_adapter
    rclcpp
    rclcpp_action
    nav2_msgs
    robot_sdk_interfaces
    robot_sdk_core
)
install(TARGETS nav2_adapter DESTINATION lib/${PROJECT_NAME})

add_executable(motion_adapter src/motion_adapter.cpp)
ament_target_dependencies(motion_adapter
    rclcpp
    rclcpp_action
    control_msgs
    sensor_msgs
    robot_sdk_interfaces
    robot_sdk_core
)
install(TARGETS motion_adapter DESTINATION lib/${PROJECT_NAME})

add_executable(moveit_adapter src/moveit_adapter.cpp)
ament_target_dependencies(moveit_adapter
    rclcpp
    rclcpp_action
    moveit_msgs
    shape_msgs
    geometry_msgs
    tf2_ros
    robot_sdk_interfaces
    robot_sdk_core
)
install(TARGETS moveit_adapter DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
