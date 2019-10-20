cmake_minimum_required(VERSION 3.5)

project(dolly_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  worlds rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
