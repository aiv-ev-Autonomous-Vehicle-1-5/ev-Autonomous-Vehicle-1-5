cmake_minimum_required(VERSION 3.14)
project(cluster_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lidar_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(filter_component SHARED
  src/filter_component.cpp
)

target_include_directories(filter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(filter_component PUBLIC cxx_std_17)

ament_target_dependencies(filter_component
  rclcpp
  rclcpp_components
  sensor_msgs
  visualization_msgs
  lidar_interfaces
)

target_link_libraries(filter_component
  Eigen3::Eigen
)

rclcpp_components_register_nodes(filter_component "cluster_filter::FilterComponent")

install(TARGETS filter_component
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
ament_export_libraries(filter_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  visualization_msgs
  lidar_interfaces
)

ament_package()
