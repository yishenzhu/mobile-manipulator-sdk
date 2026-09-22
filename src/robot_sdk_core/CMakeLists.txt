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
find_package(tf2 REQUIRED)

# INTERFACE 库：本身不产出 .so，只负责把 include 路径"传播"给使用者。
# 等后面加了 .cpp，再换成 SHARED 库。
add_library(robot_sdk_core INTERFACE)
target_include_directories(robot_sdk_core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# INTERFACE 库用 INTERFACE 作用域把依赖传播给使用者
ament_target_dependencies(robot_sdk_core INTERFACE geometry_msgs tf2)
ament_export_dependencies(geometry_msgs tf2)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

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
endif()

ament_package()
