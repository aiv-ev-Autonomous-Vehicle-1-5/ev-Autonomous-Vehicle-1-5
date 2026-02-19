cmake_minimum_required(VERSION 3.8)
project(lidar_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lidar_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

add_library(tracking_component SHARED
  src/kalman_filter.cpp
  src/byte_tracker.cpp
  src/tracking_node.cpp
)

target_include_directories(tracking_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

ament_target_dependencies(tracking_component
  rclcpp
  rclcpp_components
  lidar_interfaces
  geometry_msgs
  std_msgs
  visualization_msgs
)

target_link_libraries(tracking_component
  Eigen3::Eigen
)

rclcpp_components_register_nodes(tracking_component "lidar_tracking::TrackingNode")

install(TARGETS tracking_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(tracking_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  lidar_interfaces
  geometry_msgs
  std_msgs
  visualization_msgs
)

ament_package()
