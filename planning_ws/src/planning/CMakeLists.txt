cmake_minimum_required(VERSION 3.8)
project(planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(
  include
)

add_executable(
  planning_node
  src/planning_node.cpp
  src/hybrid_astar.cpp
  src/collision_checker.cpp
)

ament_target_dependencies(
  planning_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

add_executable(
  fake_cone_pub
  src/fake_cone_pub.cpp
)

ament_target_dependencies(
  fake_cone_pub
  rclcpp
  sensor_msgs
)

install(TARGETS
  planning_node
  fake_cone_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()