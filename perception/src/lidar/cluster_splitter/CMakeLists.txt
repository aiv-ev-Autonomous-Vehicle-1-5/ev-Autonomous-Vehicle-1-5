cmake_minimum_required(VERSION 3.8)
project(cluster_splitter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(cluster_splitter_component SHARED
  src/cluster_splitter_node.cpp
)

ament_target_dependencies(cluster_splitter_component
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_node(cluster_splitter_component
  PLUGIN "cluster_splitter::ClusterSplitterNode"
  EXECUTABLE cluster_splitter_node
)

install(TARGETS cluster_splitter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
