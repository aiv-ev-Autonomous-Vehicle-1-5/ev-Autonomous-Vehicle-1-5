cmake_minimum_required(VERSION 3.8)
project(gps_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(utm_localizer_lib
  src/utm_localizer.cpp
)

ament_target_dependencies(utm_localizer_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

add_executable(utm_localizer_node
  src/utm_localizer_node.cpp
)

target_link_libraries(utm_localizer_node
  utm_localizer_lib
)

ament_target_dependencies(utm_localizer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

# ---- waypoint_recorder ----
add_library(waypoint_recorder_lib
  src/waypoint_recorder.cpp
)

ament_target_dependencies(waypoint_recorder_lib
  rclcpp
  sensor_msgs
)

add_executable(waypoint_recorder_node
  src/waypoint_recorder_node.cpp
)

target_link_libraries(waypoint_recorder_node
  waypoint_recorder_lib
)

ament_target_dependencies(waypoint_recorder_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  utm_localizer_lib
  utm_localizer_node
  waypoint_recorder_lib
  waypoint_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()