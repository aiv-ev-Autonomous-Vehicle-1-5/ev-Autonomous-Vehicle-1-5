cmake_minimum_required(VERSION 3.16)
project(dbscan_clustering LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── CUDA auto-detection ──
# CUDA가 있으면 GPU 이웃 검색, 없으면 CPU KD-Tree fallback
# 최신 CUDA toolkit 자동 탐색 (sm_90 PTX 지원을 위해 12.x 우선)
if(NOT CMAKE_CUDA_COMPILER)
  find_program(_nvcc_latest nvcc
    PATHS /usr/local/cuda-12.5/bin /usr/local/cuda-12/bin /usr/local/cuda/bin
    NO_DEFAULT_PATH)
  if(_nvcc_latest)
    set(CMAKE_CUDA_COMPILER "${_nvcc_latest}")
    message(STATUS "[dbscan_clustering] Found newer nvcc: ${_nvcc_latest}")
  endif()
endif()
include(CheckLanguage)
check_language(CUDA)

if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  set(DBSCAN_USE_CUDA ON)
  message(STATUS "[dbscan_clustering] CUDA found — GPU neighbor search enabled")

  if (COMMAND cmake_policy)
    cmake_policy(SET CMP0104 NEW)
  endif()
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(_dbscan_cuda_architectures 60-real 61-real 70-real 75-real 80-real 86-real)
    if(CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL 11.8)
      list(APPEND _dbscan_cuda_architectures 90)
    endif()
    set(CMAKE_CUDA_ARCHITECTURES "${_dbscan_cuda_architectures}" CACHE STRING "CUDA architectures" FORCE)
  endif()
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
else()
  set(DBSCAN_USE_CUDA OFF)
  message(STATUS "[dbscan_clustering] CUDA not found — CPU KD-Tree fallback enabled")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(include)

# Component shared library
add_library(dbscan_component SHARED src/dbscan_node.cpp)
target_include_directories(dbscan_component PRIVATE include)
ament_target_dependencies(dbscan_component rclcpp rclcpp_components sensor_msgs std_msgs)

if(DBSCAN_USE_CUDA)
  # GPU kernels library (기존 코드 그대로)
  add_library(dbscan_gpu SHARED src/dbscan_gpu.cu)
  target_include_directories(dbscan_gpu PRIVATE include)
  set_target_properties(dbscan_gpu PROPERTIES POSITION_INDEPENDENT_CODE ON)
  set_target_properties(dbscan_gpu PROPERTIES CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
  target_link_libraries(dbscan_component dbscan_gpu)
  target_compile_definitions(dbscan_component PRIVATE DBSCAN_USE_CUDA)

  install(TARGETS dbscan_component dbscan_gpu
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
else()
  # CPU fallback library (nanoflann KD-Tree)
  add_library(dbscan_cpu SHARED src/dbscan_cpu.cpp)
  target_include_directories(dbscan_cpu PRIVATE include)
  set_target_properties(dbscan_cpu PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(dbscan_component dbscan_cpu)

  install(TARGETS dbscan_component dbscan_cpu
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

rclcpp_components_register_node(dbscan_component
  PLUGIN "dbscan_clustering::DBSCANNode"
  EXECUTABLE dbscan_node)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
