# ============================================================================
# CMakeLists.txt — chaining_CDT 패키지 빌드 설정
#
# 빌드 결과물:
#   1. chaining_cdt_chainer     (공유 라이브러리) — DirectionChainer (경계점 체이닝)
#   2. chaining_cdt_planner    (공유 라이브러리) — CDT 기반 Centerline 추출
#   3. chaining_cdt_postprocess (공유 라이브러리) — 경로 후처리기
#   4. lc_planner_component    (ComposableNode)  — 메인 노드 (위 3개 링크)
#
# 빌드 명령:
#   cd ~/ev_ws/planning && colcon build --packages-select chaining_CDT
# ============================================================================
cmake_minimum_required(VERSION 3.14)
project(chaining_CDT)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── CDT 라이브러리 (artem-ogre/CDT, MPL-2.0) ──
# Constrained Delaunay Triangulation 라이브러리.
# header-only 모드로 사용 (컴파일 라이브러리 불필요).
# SOURCE_SUBDIR CDT: 리포지토리 루트가 아닌 CDT/ 서브디렉토리에 CMakeLists.txt가 있음.
include(FetchContent)
FetchContent_Declare(CDT
  GIT_REPOSITORY https://github.com/artem-ogre/CDT.git
  GIT_TAG        master
  SOURCE_SUBDIR  CDT
)
set(CDT_USE_AS_COMPILED_LIBRARY OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(CDT)

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
# Library 1: chaining_cdt_chainer — DirectionChainer 라이브러리
# ============================================================================
add_library(chaining_cdt_chainer SHARED
  src/chainer/direction_chainer.cpp
)
ament_target_dependencies(chaining_cdt_chainer
  rclcpp geometry_msgs
)
install(TARGETS chaining_cdt_chainer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 2: chaining_cdt_planner — CDT 기반 Centerline 추출 라이브러리
# ============================================================================
add_library(chaining_cdt_planner SHARED
  src/planner/cdt_centerline.cpp
)
target_link_libraries(chaining_cdt_planner CDT::CDT)
ament_target_dependencies(chaining_cdt_planner
  rclcpp geometry_msgs
)
install(TARGETS chaining_cdt_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Library 4: chaining_cdt_postprocess — 경로 후처리 라이브러리
# ============================================================================
add_library(chaining_cdt_postprocess SHARED
  src/postprocess/path_postprocessor.cpp
)
ament_target_dependencies(chaining_cdt_postprocess
  rclcpp geometry_msgs ev_msgs
)
install(TARGETS chaining_cdt_postprocess
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Component: lc_planner_component — 메인 노드 (ComposableNode)
# ============================================================================
add_library(lc_planner_component SHARED
  src/nodes/chaining_CDT_node.cpp
)
target_link_libraries(lc_planner_component
  chaining_cdt_chainer
  chaining_cdt_planner
  chaining_cdt_postprocess
)
ament_target_dependencies(lc_planner_component
  rclcpp rclcpp_components
  nav_msgs geometry_msgs std_msgs ev_msgs visualization_msgs
)

rclcpp_components_register_node(lc_planner_component
  PLUGIN "chaining_CDT::LCPlannerNode"
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
