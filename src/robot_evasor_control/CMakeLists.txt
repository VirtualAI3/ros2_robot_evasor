cmake_minimum_required(VERSION 3.8)
project(robot_evasor_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  robot_evasor_control/odom_to_tf.py
  robot_evasor_control/diagnostics_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
