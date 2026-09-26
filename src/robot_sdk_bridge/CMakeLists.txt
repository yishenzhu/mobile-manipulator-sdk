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
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
