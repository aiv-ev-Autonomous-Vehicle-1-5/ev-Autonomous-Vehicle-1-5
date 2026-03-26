# ============================================================================
# CMakeLists.txt — yolo_lane_cluster 패키지 빌드 설정
#
# 빌드 결과물:
#   yolo_lane_cluster_component (ComposableNode) — 차선 전처리 노드
#     시드 기반 좌/우 판별 + 가상 차선 생성
#
# 빌드 명령:
#   cd ~/ev-Autonomous-Vehicle-1-5 && colcon build --symlink-install --packages-select yolo_lane_cluster
# ============================================================================
cmake_minimum_required(VERSION 3.14)
project(yolo_lane_cluster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS 2 의존성 패키지 찾기 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ev_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# ============================================================================
# Component: yolo_lane_cluster_component — 차선 전처리 노드 (ComposableNode)
# ============================================================================
add_library(yolo_lane_cluster_component SHARED
  src/yolo_lane_cluster_node.cpp
  src/seed_tracker.cpp
  src/virtual_lane_gen.cpp
  src/debug_publisher.cpp
)
ament_target_dependencies(yolo_lane_cluster_component
  rclcpp rclcpp_components
  geometry_msgs std_msgs ev_msgs visualization_msgs
)

rclcpp_components_register_node(yolo_lane_cluster_component
  PLUGIN "yolo_lane_cluster::YoloLaneClusterNode"
  EXECUTABLE yolo_lane_cluster_node_exe
)

install(TARGETS yolo_lane_cluster_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ── 헤더, 설정, 런치 파일 설치 ──
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# ── 테스트 설정 ──
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
