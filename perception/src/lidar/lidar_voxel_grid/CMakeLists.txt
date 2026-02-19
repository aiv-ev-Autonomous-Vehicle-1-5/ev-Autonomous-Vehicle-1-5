cmake_minimum_required(VERSION 3.8)
project(lidar_voxel_grid)
if(POLICY CMP0074)                                                                              
  cmake_policy(SET CMP0074 NEW)                                                              
endif() 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(voxel_grid_component SHARED
  src/voxel_grid_component.cpp
)
target_link_libraries(voxel_grid_component
  ${PCL_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
)
ament_target_dependencies(voxel_grid_component
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
)

rclcpp_components_register_node(voxel_grid_component
  PLUGIN "lidar_voxel_grid::VoxelGridComponent"
  EXECUTABLE voxel_grid_node
)

install(TARGETS voxel_grid_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
