cmake_minimum_required(VERSION 3.16)
project(dbscan_clustering LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure CUDA language standard and architectures to avoid CMake errors/warnings
if (COMMAND cmake_policy)
  cmake_policy(SET CMP0104 NEW)
endif()
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 60 61 70 75 80 86 CACHE STRING "CUDA architectures" FORCE)
endif()
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(include)

# CUDA GPU kernels library
add_library(dbscan_gpu SHARED src/dbscan_gpu.cu)
target_include_directories(dbscan_gpu PRIVATE include)
set_target_properties(dbscan_gpu PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(dbscan_gpu PROPERTIES CUDA_ARCHITECTURES "60;61;70;75;80;86")

# Component shared library
add_library(dbscan_component SHARED src/dbscan_node.cpp)
target_include_directories(dbscan_component PRIVATE include)
target_link_libraries(dbscan_component dbscan_gpu)
ament_target_dependencies(dbscan_component rclcpp rclcpp_components sensor_msgs std_msgs)

rclcpp_components_register_node(dbscan_component
  PLUGIN "dbscan_clustering::DBSCANNode"
  EXECUTABLE dbscan_node)

install(TARGETS dbscan_component dbscan_gpu
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()