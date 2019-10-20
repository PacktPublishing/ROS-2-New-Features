cmake_minimum_required(VERSION 3.5)

project(dolly_follow)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(dolly_follow src/dolly_follow.cpp)
add_executable(dolly_go_to_goal src/dolly_gtg.cpp)

ament_target_dependencies(dolly_follow
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs")

ament_target_dependencies(dolly_go_to_goal
  "rclcpp"
  "geometry_msgs"
	"nav_msgs"
  "sensor_msgs"
	"tf2")

install(TARGETS dolly_follow dolly_go_to_goal
  DESTINATION lib/${PROJECT_NAME})

ament_package()
