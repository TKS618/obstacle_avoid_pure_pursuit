cmake_minimum_required(VERSION 3.8)
project(obstacle_avoid_pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(obstacle_avoid_pure_pursuit_component SHARED
  src/obstacle_avoid_pure_pursuit.cpp
)

ament_target_dependencies(
  obstacle_avoid_pure_pursuit_component
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
)

rclcpp_components_register_nodes(
  obstacle_avoid_pure_pursuit_component
  "ObstacleAvoidPurePursuit"
)

install(TARGETS
  obstacle_avoid_pure_pursuit_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
