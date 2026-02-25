// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_msgs:msg/PlannerStatus.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__STRUCT_H_
#define TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OK'.
/**
  * 상태 상수 정의
  * 정상 — 경로 유효, target_speed > 0
 */
enum
{
  track_msgs__msg__PlannerStatus__OK = 0
};

/// Constant 'STOP'.
/**
  * 정지 — 경로 생성 실패
 */
enum
{
  track_msgs__msg__PlannerStatus__STOP = 1
};

/// Constant 'INFEASIBLE'.
/**
  * 실현불가 — 곡률 초과 (차량 한계)
 */
enum
{
  track_msgs__msg__PlannerStatus__INFEASIBLE = 2
};

/// Constant 'STALE'.
/**
  * 만료 — 입력 데이터 지연 (perception_ms 초과)
 */
enum
{
  track_msgs__msg__PlannerStatus__STALE = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/PlannerStatus in the package track_msgs.
/**
  * =============================================================
  * track_msgs/msg/PlannerStatus.msg
  * =============================================================
  * 플래너 상태 출력 — 로컬 플래너 파이프라인의 현재 상태를 컨트롤러에 전달
  *
  * 토픽: /planning/status
  * 발행: LocalPlannerNode (on_timer 끝에서 매 프레임 발행)
  * 구독: Controller 노드 (속도/조향 명령 결정에 사용)
  *
  * 상태 우선순위 (높은 것부터):
  *   STALE (3)      — 입력 데이터 지연/누락 → 감속 후 정지
  *   INFEASIBLE (2) — 경로 곡률 > 차량 조향 한계 → 추종 불가, 정지
  *   STOP (1)       — 유효한 경로 없음 → 정지
  *   OK (0)         — 정상 주행 가능 → target_speed 적용
  *
  * reason 필드 예시:
  *   "ok"                         — 정상
  *   "input_stale"                — 입력 데이터 타임아웃 초과
  *   "no_valid_path"              — 경로 생성 실패
  *   "curvature_exceeds_r_min"    — 곡률이 최소 회전 반경 초과
  * =============================================================
 */
typedef struct track_msgs__msg__PlannerStatus
{
  /// 타임스탬프 + 좌표계 프레임 (base_link)
  std_msgs__msg__Header header;
  /// 현재 플래너 상태 (위 상수 중 하나)
  uint8_t status;
  /// 사람이 읽을 수 있는 상태 설명 문자열 (디버깅/로깅용)
  rosidl_runtime_c__String reason;
} track_msgs__msg__PlannerStatus;

// Struct for a sequence of track_msgs__msg__PlannerStatus.
typedef struct track_msgs__msg__PlannerStatus__Sequence
{
  track_msgs__msg__PlannerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_msgs__msg__PlannerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__STRUCT_H_
