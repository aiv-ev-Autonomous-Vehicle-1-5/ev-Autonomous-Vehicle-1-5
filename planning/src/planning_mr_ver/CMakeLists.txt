# ============================================================================
# CMakeLists.txt — planning_mr_ver 패키지 빌드 설정
#
# 빌드 결과물:
#   1. planning_mr_costmap    (공유 라이브러리) — costmap 생성기
#   2. planning_mr_planner    (공유 라이브러리) — Greedy 전진 탐색 플래너
#   3. planning_mr_postprocess (공유 라이브러리) — 경로 후처리기
#   4. mr_planner_component   (ComposableNode)  — 메인 노드 (위 3개 링크)
#      → mr_planner_node_exe  (단독 실행 가능 바이너리)
#
# 빌드 명령:
#   cd ~/ev_ws/planning && colcon build --packages-select planning_mr_ver
# ============================================================================
cmake_minimum_required(VERSION 3.8)
project(planning_mr_ver)

# 컴파일러 경고 옵션 (GCC/Clang 공통)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 표준 사용 (ROS 2 Humble 권장)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS 2 의존성 패키지 찾기 ──
# ament_cmake:       ROS 2 빌드 시스템
# rclcpp:            ROS 2 C++ 클라이언트 라이브러리
# rclcpp_components: ComposableNode 등록 매크로
# nav_msgs:          Path, OccupancyGrid 메시지
# geometry_msgs:     PoseStamped, Point 메시지
# std_msgs:          Bool 등 기본 메시지
# track_msgs:        LaneBoundaryArray, ConeArray, PlannerStatus (커스텀)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(track_msgs REQUIRED)
find_package(ev_msgs REQUIRED)

# 헤더 파일 경로 (include/ 디렉토리)
include_directories(include)

# ============================================================================
# Library 1: planning_mr_costmap — costmap 생성 라이브러리
#
# CostmapGenerator 클래스를 포함.
# 콘/차선을 S극 자석으로 모델링하여 2D costmap 그리드를 생성한다.
# ============================================================================
add_library(planning_mr_costmap SHARED
  src/costmap/costmap_generator.cpp
)
ament_target_dependencies(planning_mr_costmap
  rclcpp nav_msgs geometry_msgs track_msgs ev_msgs
)
install(TARGETS planning_mr_costmap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 2: planning_mr_planner — Greedy 전진 탐색 플래너 라이브러리
#
# MagneticPlanner 클래스를 포함.
# costmap 위에서 전방 180° 최소 cost 탐색으로 raw_path를 생성한다.
# ============================================================================
add_library(planning_mr_planner SHARED
  src/planner/magnetic_planner.cpp
)
ament_target_dependencies(planning_mr_planner
  rclcpp geometry_msgs track_msgs ev_msgs
)
install(TARGETS planning_mr_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 3: planning_mr_postprocess — 경로 후처리 라이브러리
#
# PathPostprocessor 클래스를 포함.
# prune → smooth → resample → yaw 4단계 후처리를 수행한다.
# ============================================================================
add_library(planning_mr_postprocess SHARED
  src/postprocess/path_postprocessor.cpp
)
ament_target_dependencies(planning_mr_postprocess
  rclcpp geometry_msgs track_msgs ev_msgs
)
install(TARGETS planning_mr_postprocess
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Component: mr_planner_component — 메인 노드 (ComposableNode)
#
# MRPlannerNode 클래스를 포함.
# 위 3개 라이브러리를 링크하여 전체 파이프라인을 오케스트레이션한다.
#
# rclcpp_components_register_node(): 이 매크로로 인해:
#   1. component_container에서 동적 로딩 가능 (Intra-process 통신)
#   2. mr_planner_node_exe 단독 실행 바이너리도 자동 생성
# ============================================================================
add_library(mr_planner_component SHARED
  src/nodes/mr_planner_node.cpp
)
# target_link_libraries: 위에서 만든 3개 라이브러리와 링크
target_link_libraries(mr_planner_component
  planning_mr_costmap
  planning_mr_planner
  planning_mr_postprocess
)
ament_target_dependencies(mr_planner_component
  rclcpp rclcpp_components
  nav_msgs geometry_msgs std_msgs track_msgs ev_msgs
)

# ComposableNode 등록 + 단독 실행 바이너리 생성
rclcpp_components_register_node(mr_planner_component
  PLUGIN "planning_mr_ver::MRPlannerNode"
  EXECUTABLE mr_planner_node_exe
)

install(TARGETS mr_planner_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ── 헤더, 설정, 런치 파일 설치 ──
install(DIRECTORY include/ DESTINATION include)        # 헤더 파일
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})  # planning_mr.yaml
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})  # launch 파일

# ── 테스트 설정 (ament_lint) ──
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)    # copyright 검사 비활성화
  set(ament_cmake_cpplint_FOUND TRUE)      # cpplint 검사 비활성화
  ament_lint_auto_find_test_dependencies()
endif()

# ament 패키지 등록 (colcon이 이 패키지를 인식하기 위해 필수)
ament_package()
