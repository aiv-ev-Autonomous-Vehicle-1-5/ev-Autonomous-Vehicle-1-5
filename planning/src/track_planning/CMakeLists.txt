# ============================================================================
# track_planning 패키지 — CMakeLists.txt
# ============================================================================
# 로컬 플래너(Local Planner) 파이프라인 빌드 설정
#
# 빌드 구조 (2개 공유 라이브러리 + 1개 컴포넌트):
#   1) track_planning_corridor   — 복도(corridor) 구축, 가상 경계, DTR 센터라인
#   2) track_planning_postprocess — 경로 후처리 (가지치기, 스무딩, 리샘플)
#   3) local_planner_component   — ROS 2 ComposableNode (위 2개 라이브러리 링크)
#
# 의존성 흐름:
#   local_planner_component
#     └── track_planning_corridor
#     └── track_planning_postprocess
# ============================================================================

cmake_minimum_required(VERSION 3.8)   # CMake 최소 버전 요구
project(track_planning)               # 패키지(프로젝트) 이름 선언

# --- 컴파일러 경고 옵션 ---
# GCC 또는 Clang 사용 시 엄격한 경고 활성화
#   -Wall     : 일반적인 경고 전부
#   -Wextra   : 추가 경고
#   -Wpedantic: ISO C++ 표준 위반 경고
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- C++ 표준 설정 ---
set(CMAKE_CXX_STANDARD 17)           # C++17 사용 (structured bindings, std::optional 등)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # C++17 미지원 컴파일러에서 빌드 실패 강제

# ============================================================================
# 의존성 패키지 탐색 (find_package)
# ============================================================================
# ament_cmake         : ROS 2 빌드 시스템 기본 매크로
# rclcpp              : ROS 2 C++ 클라이언트 라이브러리 (노드, 퍼블리셔, 서브스크립션 등)
# rclcpp_components   : ComposableNode 등록/로딩 매크로
# nav_msgs            : OccupancyGrid, Path 등 내비게이션 메시지
# geometry_msgs       : Point, Pose, PoseStamped 등 기하학 메시지
# std_msgs            : Bool, String, Header 등 기본 메시지
# visualization_msgs  : Marker, MarkerArray 등 RViz2 시각화 메시지
# track_msgs          : 커스텀 메시지 (Cone, LaneBoundary, Obstacle, PlannerStatus 등)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(track_msgs REQUIRED)

# 헤더 파일 검색 경로 추가 (include/ 디렉토리)
include_directories(include)

# ============================================================================
# 라이브러리 1: track_planning_corridor (복도 구축 모듈)
# ============================================================================
# 파이프라인 Phase 1+2: 차선 경계로부터 복도(corridor) 구축
#   - corridor_builder.cpp    : Greedy Chaining 알고리즘으로 좌/우 경계 폴리라인 생성
#   - virtual_boundary.cpp    : 한쪽 경계만 있을 때 가상 경계 생성
#   - centerline_builder.cpp  : DTR 외심 기반 또는 한쪽 오프셋으로 센터라인 생성
add_library(track_planning_corridor SHARED
  src/corridor/corridor_builder.cpp
  src/corridor/virtual_boundary.cpp
  src/corridor/centerline_builder.cpp
)

ament_target_dependencies(track_planning_corridor
  rclcpp
  nav_msgs
  geometry_msgs
  track_msgs
)

# SHARED 라이브러리 설치 (lib/ 디렉토리)
#   ARCHIVE: 정적 라이브러리 (.a)  — 공유 라이브러리에서는 import library 역할
#   LIBRARY: 공유 라이브러리 (.so) — 런타임 로딩 대상
#   RUNTIME: 실행 파일            — Windows DLL용 (Linux에서는 해당 없음)
install(TARGETS track_planning_corridor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# 라이브러리 2: track_planning_postprocess (후처리 모듈)
# ============================================================================
# 파이프라인 Phase 4: 경로 후처리
#   - path_postprocessor.cpp : Greedy Shortcut 가지치기 → 이동 평균 스무딩
#                               → 등간격 리샘플 → Yaw 각도 계산
add_library(track_planning_postprocess SHARED
  src/postprocess/path_postprocessor.cpp
)

ament_target_dependencies(track_planning_postprocess
  rclcpp
  geometry_msgs
  track_msgs
)

install(TARGETS track_planning_postprocess
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# 컴포넌트: local_planner_component (ROS 2 ComposableNode)
# ============================================================================
# 모든 모듈을 통합하는 메인 노드
#   - local_planner_node.cpp : LocalPlannerNode 클래스
#     → 타이머 콜백(on_timer)에서 전체 파이프라인 실행:
#       Stale 검사 → 입력 파싱 → 복도 구축 → 가상 경계 →
#       DTR 센터라인 → 후처리 → 안전 검사 → 퍼블리시
#
# ComposableNode로 등록하면:
#   - launch 파일에서 ComposableNodeContainer에 로딩 가능
#   - Intra-process 통신(Zero-copy) 활성화 가능 → 메모리 복사 오버헤드 제거
add_library(local_planner_component SHARED
  src/nodes/local_planner_node.cpp
)

# 파이프라인 라이브러리를 직접 링크 (CMake target_link_libraries)
# ament_target_dependencies는 ROS 2 패키지 의존성만 처리하므로,
# 같은 패키지 내 라이브러리는 target_link_libraries로 연결해야 함
target_link_libraries(local_planner_component
  track_planning_corridor
  track_planning_postprocess
)

# ROS 2 패키지 의존성 설정 (include 경로 + 링크 라이브러리 자동 처리)
ament_target_dependencies(local_planner_component
  rclcpp
  rclcpp_components              # ComposableNode 등록 매크로 제공
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  track_msgs
)

# ComposableNode 등록 매크로
#   PLUGIN    : 클래스 전체 이름 (네임스페이스::클래스명)
#   EXECUTABLE: 단독 실행 가능한 바이너리 이름 (ros2 run track_planning local_planner_node_exe)
# → pluginlib에 등록되어 launch 파일의 ComposableNodeContainer에서 로딩 가능
rclcpp_components_register_node(local_planner_component
  PLUGIN "track_planning::LocalPlannerNode"
  EXECUTABLE local_planner_node_exe
)

install(TARGETS local_planner_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# 설치: 헤더, 설정 파일, launch 파일
# ============================================================================

# 헤더 파일 설치 (include/ → install 경로의 include/)
# → 다른 패키지에서 이 패키지의 헤더를 참조할 수 있도록 함
install(DIRECTORY include/
  DESTINATION include
)

# 설정 파일 설치 (config/ → share/track_planning/config/)
# → planning.yaml 파라미터 파일 포함
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# launch 파일 설치 (launch/ → share/track_planning/launch/)
# → ros2 launch track_planning planning.launch.py 명령으로 실행 가능
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# 테스트 설정 (colcon test 시 활성화)
# ============================================================================
# BUILD_TESTING이 ON일 때만 테스트 의존성 탐색
# copyright, cpplint 검사는 건너뜀 (FOUND TRUE로 설정)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament 패키지 등록 마무리 (package.xml과 연동)
# → 다른 패키지에서 find_package(track_planning)로 찾을 수 있게 함
ament_package()
