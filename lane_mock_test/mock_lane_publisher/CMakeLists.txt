cmake_minimum_required(VERSION 3.8)
project(mock_lane_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ev_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ── mock_lane_node ──
add_executable(mock_lane_node src/mock_lane_node.cpp)
ament_target_dependencies(mock_lane_node
  rclcpp
  ev_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS mock_lane_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
