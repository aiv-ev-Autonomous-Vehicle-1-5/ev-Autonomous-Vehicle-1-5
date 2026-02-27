cmake_minimum_required(VERSION 3.8)
project(make_bbox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ev_msgs REQUIRED)

include_directories(include)

add_library(make_bbox_component SHARED
  src/make_bbox_node.cpp
)
ament_target_dependencies(make_bbox_component
  rclcpp rclcpp_components sensor_msgs visualization_msgs ev_msgs
)

rclcpp_components_register_node(make_bbox_component
  PLUGIN "make_bbox::MakeBBoxNode"
  EXECUTABLE make_bbox_node
)

install(TARGETS make_bbox_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
