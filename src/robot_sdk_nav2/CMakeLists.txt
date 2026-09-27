cmake_minimum_required(VERSION 3.8)

# 只有配置和 launch，不编译代码。
# 构建时那条 "...CMAKE_EXPORT_COMPILE_COMMANDS" 告警来自本机 colcon 传的 -D、本包无编译单元；
# 包内消不掉，由同目录 colcon.pkg 传 --no-warn-unused-cli 解决。
project(robot_sdk_nav2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})

ament_package()
