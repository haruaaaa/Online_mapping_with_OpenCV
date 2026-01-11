cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(PROGRAMS
  scripts/publish_urdf_topic.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

