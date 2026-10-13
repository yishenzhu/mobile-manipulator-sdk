cmake_minimum_required(VERSION 3.8)
project(robot_sdk_core)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 把编译器告警开到"能发现问题"的程度。
# 关键在 -Wall 里的 -Wswitch：它会检查 switch 是否覆盖了枚举的所有取值。
# 这正是 ErrorCode 的 toString() 故意不写 default 分支的原因。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_sdk_interfaces REQUIRED)

# 有 .cpp 了，所以从 INTERFACE 库升级为真正的 SHARED 库（会产出 .so）。
# INTERFACE 库：不产出二进制，只负责传播 include 路径（纯头文件库）。
# SHARED 库：需要编译源码，产出 librobot_sdk_core.so。
add_library(robot_sdk_core SHARED
    src/config.cpp
    src/runtime.cpp
    src/transform.cpp
    src/perception_api.cpp
    src/status_api.cpp
    src/navigation_api.cpp
    src/motion_api.cpp
    src/robot.cpp
)
target_compile_features(robot_sdk_core PUBLIC cxx_std_17)
# PUBLIC = 编译这个库自己要用 + 下游使用者也要用
target_include_directories(robot_sdk_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_sdk_core
    geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs rclcpp rclcpp_action
    robot_sdk_interfaces)

# install(TARGETS) 把编译产物装进 install/；
# EXPORT 让别的包能通过 find_package 找到并链接它。
install(TARGETS robot_sdk_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# HAS_LIBRARY_TARGET 是必须的：没有它，下游只拿到 include 路径，
# 链接时会 undefined reference。
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
    geometry_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs rclcpp rclcpp_action
    robot_sdk_interfaces)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_error test/test_error.cpp)
    target_link_libraries(test_error robot_sdk_core)
    ament_add_gtest(test_result test/test_result.cpp)
    target_link_libraries(test_result robot_sdk_core)
    ament_add_gtest(test_types test/test_types.cpp)
    target_link_libraries(test_types robot_sdk_core)
    ament_add_gtest(test_latest_value_cache test/test_latest_value_cache.cpp)
    target_link_libraries(test_latest_value_cache robot_sdk_core)
    ament_add_gtest(test_config test/test_config.cpp)
    target_link_libraries(test_config robot_sdk_core)
    ament_add_gtest(test_transform test/test_transform.cpp)
    target_link_libraries(test_transform robot_sdk_core)
    # 这条顺带充当 action_util.hpp 的自包含性测试 —— 它不 include 任何别的 SDK 头文件
    ament_add_gtest(test_action_util test/test_action_util.cpp)
    target_link_libraries(test_action_util robot_sdk_core)
    ament_add_gtest(test_perception_helpers test/test_perception_helpers.cpp)
    target_link_libraries(test_perception_helpers robot_sdk_core)
    ament_add_gtest(test_joint_wrap test/test_joint_wrap.cpp)
    target_link_libraries(test_joint_wrap robot_sdk_core)
    # QoS 的字符串→枚举映射 + "默认值就是文档契约"这两条都在这里
    ament_add_gtest(test_subscription_qos test/test_subscription_qos.cpp)
    target_link_libraries(test_subscription_qos robot_sdk_core)
    ament_add_gtest(test_navigation_api test/test_navigation_api.cpp)
    target_link_libraries(test_navigation_api robot_sdk_core)
    ament_add_gtest(test_motion_api test/test_motion_api.cpp)
    target_link_libraries(test_motion_api robot_sdk_core)
    # 这条顺带充当 robot_sdk.hpp（umbrella）的自包含性测试
    ament_add_gtest(test_robot test/test_robot.cpp)
    target_link_libraries(test_robot robot_sdk_core)
endif()

ament_package()
