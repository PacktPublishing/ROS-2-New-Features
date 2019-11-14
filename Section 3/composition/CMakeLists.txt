cmake_minimum_required(VERSION 3.5)
project(composition)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# Create a SHARED LIBRARY
add_library(talker_component SHARED
  src/talker_component.cpp)
target_compile_definitions(talker_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(talker_component
  "rclcpp"
  # "rclcpp_components"
	"class_loader"
  "std_msgs")
# rclcpp_components_register_nodes(talker_component "composition::Talker")
# Register the library with the pluginlib
rclcpp_register_node_plugins(talker_component "composition::Talker")
set(node_plugins "${node_plugins}composition::Talker;$<TARGET_FILE:talker_component>\n")

add_library(listener_component SHARED
  src/listener_component.cpp)
target_compile_definitions(listener_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(listener_component
  "rclcpp"
  # "rclcpp_components"
	"class_loader"
  "std_msgs")
# rclcpp_components_register_nodes(listener_component "composition::Listener")
rclcpp_register_node_plugins(listener_component "composition::Listener")
set(node_plugins "${node_plugins}composition::Listener;$<TARGET_FILE:listener_component>\n")

add_executable(manual_composition
  src/manual_composition.cpp)
target_link_libraries(manual_composition
  talker_component
  listener_component
)
ament_target_dependencies(manual_composition
  "rclcpp")

install(TARGETS
  talker_component
  listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME})

ament_package()
