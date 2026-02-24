# ============================================================================
# track_msgs 패키지 — CMakeLists.txt
# ============================================================================
# 로컬 플래너 파이프라인에서 사용하는 커스텀 ROS 2 메시지 정의 패키지
#
# 정의된 메시지 (8종):
#   - LaneBoundary.msg       : 단일 차선 경계 (좌/우 구분, 포인트 배열)
#   - LaneBoundaryArray.msg  : 차선 경계 배열 (여러 경계를 한 번에 전달)
#   - Cone.msg               : 단일 콘 (라바콘/교통콘, 2D 위치 + 분류)
#   - ConeArray.msg          : 콘 배열
#   - Obstacle.msg           : 단일 장애물 (PE 드럼 등, 위치 + 크기)
#   - ObstacleArray.msg      : 장애물 배열
#   - PlannerStatus.msg      : 플래너 상태 정보 (모드, 속도, 곡률 등)
#   - SystemState.msg        : 시스템 전체 상태 (FSM 상태 등)
#
# 빌드 시 rosidl이 자동으로 C++/Python 바인딩 코드를 생성함
# ============================================================================

cmake_minimum_required(VERSION 3.8)   # CMake 최소 버전 요구
project(track_msgs)                   # 패키지 이름 선언

# --- 컴파일러 경고 옵션 ---
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# 의존성 패키지 탐색
# ============================================================================
# ament_cmake               : ROS 2 빌드 시스템 기본 매크로
# rosidl_default_generators : .msg/.srv/.action 파일 → C++/Python 코드 자동 생성기
# geometry_msgs             : Point, Pose 등 — 커스텀 메시지에서 참조하는 기본 기하학 타입
# std_msgs                  : Header 등 — 타임스탬프, 프레임 ID 포함 표준 메시지
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ============================================================================
# 메시지 인터페이스 생성 (rosidl_generate_interfaces)
# ============================================================================
# .msg 파일들을 읽어서 C++ 헤더(track_msgs/msg/xxx.hpp)와
# Python 모듈(track_msgs.msg.Xxx)을 자동 생성
#
# DEPENDENCIES: 이 메시지들이 참조하는 외부 메시지 패키지
#   - geometry_msgs : Point (3D 좌표), Pose (위치+방향) 등
#   - std_msgs      : Header (stamp + frame_id) 등
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneBoundary.msg"        # 차선 경계 (side: LEFT/RIGHT, points[])
  "msg/LaneBoundaryArray.msg"   # 차선 경계 배열 (header + boundaries[])
  "msg/Cone.msg"                # 콘 (position, classification)
  "msg/ConeArray.msg"           # 콘 배열 (header + cones[])
  "msg/Obstacle.msg"            # 장애물 (position, radius)
  "msg/ObstacleArray.msg"       # 장애물 배열 (header + obstacles[])
  "msg/PlannerStatus.msg"       # 플래너 상태 (mode, speed, curvature 등)
  "msg/SystemState.msg"         # 시스템 상태 (FSM state)
  DEPENDENCIES geometry_msgs std_msgs
)

# 런타임 의존성 내보내기
# → 이 패키지의 메시지를 사용하는 다른 패키지가
#   rosidl_default_runtime을 자동으로 의존하도록 설정
ament_export_dependencies(rosidl_default_runtime)

# ============================================================================
# 테스트 설정
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)   # copyright 검사 건너뜀
  set(ament_cmake_cpplint_FOUND TRUE)     # cpplint 검사 건너뜀
  ament_lint_auto_find_test_dependencies()
endif()

# ament 패키지 등록 마무리
ament_package()
