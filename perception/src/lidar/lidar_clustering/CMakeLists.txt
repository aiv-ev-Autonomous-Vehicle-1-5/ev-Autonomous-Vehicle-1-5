cmake_minimum_required(VERSION 3.18)
project(lidar_clustering LANGUAGES CXX CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CUDA configuration
find_package(CUDAToolkit REQUIRED)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_ARCHITECTURES 89)  # Ada Lovelace (RTX 4060)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# CUDA kernels as static library (compiled by nvcc, isolated from ROS2 headers)
add_library(dbscan_gpu_kernels STATIC
  src/dbscan_gpu.cu
)
target_include_directories(dbscan_gpu_kernels PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
set_target_properties(dbscan_gpu_kernels PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)
target_link_libraries(dbscan_gpu_kernels
  CUDA::cudart
)

# Main ROS2 component library (compiled by g++)
add_library(clustering_component SHARED
  src/dbscan_gpu.cpp
  src/clustering_component.cpp
)

target_include_directories(clustering_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(clustering_component PUBLIC cxx_std_17)

ament_target_dependencies(clustering_component
  rclcpp
  rclcpp_components
  sensor_msgs
)

target_link_libraries(clustering_component
  dbscan_gpu_kernels
)

rclcpp_components_register_nodes(clustering_component "lidar_clustering::ClusteringComponent")

install(TARGETS clustering_component
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
ament_export_libraries(clustering_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
)

ament_package()
