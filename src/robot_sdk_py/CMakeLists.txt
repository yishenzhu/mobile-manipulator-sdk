cmake_minimum_required(VERSION 3.8)
project(robot_sdk_py)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 和 C++ 侧同一条纪律：警示全开（SDK 的门面，不该带着 warning 出门）。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(rclcpp REQUIRED)
find_package(robot_sdk_core REQUIRED)
find_package(robot_sdk_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(PYTHON_SITE_PACKAGES
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

# ⚠️ 这里**不能**用 ament_target_dependencies：它内部走的是 target_link_libraries 的**普通签名**，
#    而 pybind11_add_module 已经用了**关键字签名**（PRIVATE）—— 同一个 target 混用两种签名，
#    CMake 会直接报错。所以改成显式写 include/link（用 find_package 给出的那两组变量）。
pybind11_add_module(_robot_sdk NO_EXTRAS src/robot_sdk_bindings.cpp)
target_include_directories(_robot_sdk PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${robot_sdk_interfaces_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)
# ⚠️ robot_sdk_core 是**导入目标**（`ament_export_targets` 那套），要用它的全名链接 ——
#    `${robot_sdk_core_LIBRARIES}` 在这个组合下是空的，链上去会 undefined symbol。
#    它的 INTERFACE 里带着自己的 include 和传递依赖，所以不用再单独写。
target_link_libraries(_robot_sdk PRIVATE
    robot_sdk_core::robot_sdk_core
    ${robot_sdk_interfaces_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
)

# 纯 Python 那层（robot_sdk/__init__.py）走 ament_python 装；
# 编译出来的扩展模块要装进**同一个**包目录，`from ._robot_sdk import ...` 才找得到。
ament_python_install_package(robot_sdk)
install(TARGETS _robot_sdk DESTINATION ${PYTHON_SITE_PACKAGES}/robot_sdk)
install(DIRECTORY python DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    # 测试环境的 PYTHONPATH 由 ament 的环境钩子给（install 的 site-packages），不用手动加。
    ament_add_pytest_test(test_bindings test/test_bindings.py)
endif()

ament_package()
