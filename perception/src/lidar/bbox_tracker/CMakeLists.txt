cmake_minimum_required(VERSION 3.8)
project(bbox_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ev_msgs REQUIRED)
find_package(t870_msgs REQUIRED)

include_directories(include)

# 컴포넌트: tracker 노드
add_library(bbox_tracker_component SHARED
  src/bbox_tracker_node.cpp
)
ament_target_dependencies(bbox_tracker_component
  rclcpp rclcpp_components
  geometry_msgs visualization_msgs
  ev_msgs t870_msgs
)

# 컴포넌트 등록 + standalone 실행 파일
rclcpp_components_register_node(bbox_tracker_component
  PLUGIN "bbox_tracker::BBoxTrackerNode"
  EXECUTABLE bbox_tracker_node
)

install(TARGETS bbox_tracker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)

ament_package()
