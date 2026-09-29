cmake_minimum_required(VERSION 3.8)
project(robot_sdk_gazebo)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# gz 的 DiffDrive 只发 /odom 话题、不发 TF，而 Nav2 要 map→odom→base 这条完整的链。
add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster
    rclcpp
    nav_msgs
    geometry_msgs
    tf2_ros
)

install(TARGETS odom_tf_broadcaster DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config worlds DESTINATION share/${PROJECT_NAME})

ament_package()
