cmake_minimum_required(VERSION 3.8)
project(pp_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(erp42_msgs REQUIRED)

add_executable(pure_pursuit_relative_node src/pure_pursuit_relative_node.cpp)

ament_target_dependencies(pure_pursuit_relative_node
  rclcpp
  visualization_msgs
  erp42_msgs
)

install(TARGETS
  pure_pursuit_relative_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()