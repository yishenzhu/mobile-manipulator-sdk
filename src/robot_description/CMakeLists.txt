cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

# 构建期把 xacro 展开成纯 URDF 再装。消费方（mock、SDK、任何 ros2 工具）于是**不必在
# 运行期依赖 xacro** —— 装出去的 mobile_manipulator.urdf 拿来即用。
set(URDF_FILE ${CMAKE_CURRENT_BINARY_DIR}/mobile_manipulator.urdf)
add_custom_command(
  OUTPUT ${URDF_FILE}
  COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/urdf/mobile_manipulator.urdf.xacro
          -o ${URDF_FILE}
  DEPENDS urdf/mobile_manipulator.urdf.xacro
  COMMENT "展开 mobile_manipulator.urdf.xacro"
)
add_custom_target(generate_urdf ALL DEPENDS ${URDF_FILE})

# xacro 源也装出去 —— 想改机器人形状的人应该改它，而不是改展开后的产物。
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
# SRDF 也装出去 —— 它是"语义描述"（哪些关节算一组、谁是末端），MoveIt 那侧要读它。
install(DIRECTORY srdf DESTINATION share/${PROJECT_NAME})
# MoveIt 那侧的配置（kinematics.yaml）和它的 launch，一起装。
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES ${URDF_FILE} DESTINATION share/${PROJECT_NAME}/urdf)

ament_package()
