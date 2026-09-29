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
find_package(urdf REQUIRED)
find_package(moveit_msgs REQUIRED)

# 这是个可执行文件（不是库）：模拟机器人是个"程序"。
add_executable(mock_robot_node
    src/mock_robot_node.cpp
    src/mock_robot_main.cpp
    src/joint_spec.cpp
)
# 不用 $<BUILD_INTERFACE:...>：这个 include 是 PRIVATE、可执行文件也不导出，
# 所以"构建树 vs 安装树"的分歧根本不存在（那句生成器表达式只在路径要传播时才有意义）。
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
    urdf
)

# 假的下游：说 FollowJointTrajectory 的关节轨迹控制器替身，用来验运动链路。
add_executable(joint_trajectory_mock
    src/joint_trajectory_mock.cpp
    src/joint_spec.cpp
)
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
    urdf
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

# 假 move_group：只为把 moveit_adapter 的翻译摊开来看，不做任何规划。
add_executable(move_group_mock src/move_group_mock.cpp)
ament_target_dependencies(move_group_mock
    rclcpp
    rclcpp_action
    moveit_msgs
)
install(TARGETS move_group_mock DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    # joint_spec.cpp 是**编进可执行文件**的（不是库），所以测试把它再编一遍。
    ament_add_gtest(test_joint_spec
        test/test_joint_spec.cpp
        src/joint_spec.cpp
    )
    target_include_directories(test_joint_spec PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_joint_spec robot_sdk_core urdf)
endif()

ament_package()
