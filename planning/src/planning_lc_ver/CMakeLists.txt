# ============================================================================
# CMakeLists.txt — planning_lc_ver 패키지 빌드 설정
#
# 빌드 결과물:
#   1. planning_lc_chainer     (공유 라이브러리) — LineChainer (신규)
#   2. planning_lc_costmap     (공유 라이브러리) — costmap 생성기 (체인 입력)
#   3. planning_lc_planner     (공유 라이브러리) — Greedy 전진 탐색 플래너
#   4. planning_lc_postprocess (공유 라이브러리) — 경로 후처리기
#   5. lc_planner_component    (ComposableNode)  — 메인 노드 (위 4개 링크)
#
# 빌드 명령:
#   cd ~/ev_ws/planning && colcon build --packages-select planning_lc_ver
# ============================================================================
cmake_minimum_required(VERSION 3.8)
project(planning_lc_ver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS 2 의존성 패키지 찾기 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ev_msgs REQUIRED)

include_directories(include)

# ============================================================================
# Library 1: planning_lc_chainer — LineChainer 라이브러리 (신규)
# ============================================================================
add_library(planning_lc_chainer SHARED
  src/chainer/line_chainer.cpp
)
ament_target_dependencies(planning_lc_chainer
  rclcpp geometry_msgs
)
install(TARGETS planning_lc_chainer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 2: planning_lc_costmap — costmap 생성 라이브러리
# ============================================================================
add_library(planning_lc_costmap SHARED
  src/costmap/costmap_generator.cpp
)
ament_target_dependencies(planning_lc_costmap
  rclcpp nav_msgs geometry_msgs ev_msgs
)
install(TARGETS planning_lc_costmap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 3: planning_lc_planner — Greedy 전진 탐색 플래너 라이브러리
# ============================================================================
add_library(planning_lc_planner SHARED
  src/planner/magnetic_planner.cpp
)
ament_target_dependencies(planning_lc_planner
  rclcpp geometry_msgs ev_msgs
)
install(TARGETS planning_lc_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 4: planning_lc_postprocess — 경로 후처리 라이브러리
# ============================================================================
add_library(planning_lc_postprocess SHARED
  src/postprocess/path_postprocessor.cpp
)
ament_target_dependencies(planning_lc_postprocess
  rclcpp geometry_msgs ev_msgs
)
install(TARGETS planning_lc_postprocess
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Component: lc_planner_component — 메인 노드 (ComposableNode)
# ============================================================================
add_library(lc_planner_component SHARED
  src/nodes/lc_planner_node.cpp
)
target_link_libraries(lc_planner_component
  planning_lc_chainer
  planning_lc_costmap
  planning_lc_planner
  planning_lc_postprocess
)
ament_target_dependencies(lc_planner_component
  rclcpp rclcpp_components
  nav_msgs geometry_msgs std_msgs ev_msgs
)

rclcpp_components_register_node(lc_planner_component
  PLUGIN "planning_lc_ver::LCPlannerNode"
  EXECUTABLE lc_planner_node_exe
)

install(TARGETS lc_planner_component
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
