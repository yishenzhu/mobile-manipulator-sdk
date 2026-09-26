cmake_minimum_required(VERSION 3.8)
project(robot_sdk_examples)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_sdk_core REQUIRED)

# 两个示例：一个"最小可用"，一个"走一遍导航的两种用法"。
# 都只依赖 robot_sdk_core —— 客户代码就该只看到 SDK 的头文件。
foreach(example hello_robot demo_navigation)
    add_executable(${example} src/${example}.cpp)
    ament_target_dependencies(${example} rclcpp robot_sdk_core)
    install(TARGETS ${example} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
