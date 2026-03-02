cmake_minimum_required(VERSION 3.8)
project(gps_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_library(utm_localizer_lib
  src/utm_localizer.cpp
)

ament_target_dependencies(utm_localizer_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
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
  nav_msgs
)

install(TARGETS
  utm_localizer_lib
  utm_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()