# ============================================================================
# CMakeLists.txt — chaining_costmap_ver 패키지 빌드 설정
#
# 빌드 결과물:
#   1. chaining_costmap_chainer     (공유 라이브러리) — DirectionChainer (경계점 체이닝)
#   2. chaining_costmap_costmap     (공유 라이브러리) — 가우시안 비용 지도 생성
#   3. chaining_costmap_astar       (공유 라이브러리) — A* 경로 탐색
#   4. chaining_costmap_postprocess (공유 라이브러리) — 경로 후처리기
#   5. lc_planner_component    (ComposableNode)  — 메인 노드 (위 4개 링크)
#
# 빌드 명령:
#   cd ~/ev_ws/planning && colcon build --symlink-install --packages-select chaining_costmap_ver
# ============================================================================
cmake_minimum_required(VERSION 3.14)
project(chaining_costmap_ver)

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
find_package(visualization_msgs REQUIRED)

include_directories(include)

# ============================================================================
# Library 1: chaining_costmap_chainer — DirectionChainer 라이브러리
#   direction_chainer.cpp: chain() 오케스트레이터
#   seed_selector.cpp:     find_seed(), knn()
#   graph_builder.cpp:     build_graph()
#   backbone_extractor.cpp: extract_backbone(), chain_one_direction(), compute_cost()
#   chain_resampler.cpp:   resample_component()
# ============================================================================
add_library(chaining_costmap_chainer SHARED
  src/chainer/direction_chainer.cpp
  src/chainer/seed_selector.cpp
  src/chainer/graph_builder.cpp
  src/chainer/backbone_extractor.cpp
  src/chainer/chain_resampler.cpp
)
ament_target_dependencies(chaining_costmap_chainer
  rclcpp geometry_msgs
)
install(TARGETS chaining_costmap_chainer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 2: chaining_costmap_costmap — 가우시안 비용 지도 생성 라이브러리
# ============================================================================
add_library(chaining_costmap_costmap SHARED
  src/costmap/costmap_generator.cpp
)
ament_target_dependencies(chaining_costmap_costmap
  rclcpp geometry_msgs
)
install(TARGETS chaining_costmap_costmap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 3: chaining_costmap_astar — A* / Hybrid A* 경로 탐색 라이브러리
# ============================================================================
add_library(chaining_costmap_astar SHARED
  src/planner/astar_planner.cpp
  src/planner/hybrid_astar_planner.cpp
)
ament_target_dependencies(chaining_costmap_astar
  rclcpp geometry_msgs
)
install(TARGETS chaining_costmap_astar
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 4: chaining_costmap_postprocess — 경로 후처리 라이브러리
#   path_postprocessor.cpp: process() 오케스트레이터
#   prune.cpp:              ① Douglas-Peucker 유사 경로 단순화
#   smooth.cpp:             ③ 이동 평균 필터
#   curvature_clamp.cpp:    ④ 최대 곡률 제한
# ============================================================================
add_library(chaining_costmap_postprocess SHARED
  src/postprocess/path_postprocessor.cpp
  src/postprocess/prune.cpp
  src/postprocess/smooth.cpp
  src/postprocess/curvature_clamp.cpp
)
ament_target_dependencies(chaining_costmap_postprocess
  rclcpp geometry_msgs ev_msgs
)
install(TARGETS chaining_costmap_postprocess
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Component: lc_planner_component — 메인 노드 (ComposableNode)
#   chaining_costmap_ver_node.cpp: LCPlannerNode (생성자 + on_timer 파이프라인)
#   input_parser.cpp:              Stage 1 입력 파싱
#   goal_calculator.cpp:           Stage 3c goal 계산
#   debug_publisher.cpp:           Stage 7 디버그 시각화
# ============================================================================
add_library(lc_planner_component SHARED
  src/nodes/chaining_costmap_ver_node.cpp
  src/nodes/input_parser.cpp
  src/nodes/goal_calculator.cpp
  src/nodes/debug_publisher.cpp
)
target_link_libraries(lc_planner_component
  chaining_costmap_chainer
  chaining_costmap_costmap
  chaining_costmap_astar
  chaining_costmap_postprocess
)
ament_target_dependencies(lc_planner_component
  rclcpp rclcpp_components
  nav_msgs geometry_msgs std_msgs ev_msgs visualization_msgs
)

rclcpp_components_register_node(lc_planner_component
  PLUGIN "chaining_costmap_ver::LCPlannerNode"
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
