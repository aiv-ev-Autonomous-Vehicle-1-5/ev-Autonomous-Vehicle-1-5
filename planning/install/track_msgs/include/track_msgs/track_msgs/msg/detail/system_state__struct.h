// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_msgs:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_
#define TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OFFLINE'.
/**
  * 시스템 상태 상수 정의
  * 시스템 비활성
 */
enum
{
  track_msgs__msg__SystemState__OFFLINE = 0
};

/// Constant 'MANUAL'.
/**
  * 수동 조종 모드
 */
enum
{
  track_msgs__msg__SystemState__MANUAL = 1
};

/// Constant 'AUTO_STANDBY'.
/**
  * 자율주행 대기
 */
enum
{
  track_msgs__msg__SystemState__AUTO_STANDBY = 2
};

/// Constant 'AUTO_ACTIVE'.
/**
  * 자율주행 활성 (경로 추종 중)
 */
enum
{
  track_msgs__msg__SystemState__AUTO_ACTIVE = 3
};

/// Constant 'AUTO_HOLD'.
/**
  * 자율주행 일시정지
 */
enum
{
  track_msgs__msg__SystemState__AUTO_HOLD = 4
};

/// Constant 'INFEASIBLE'.
/**
  * 경로 추종 불가
 */
enum
{
  track_msgs__msg__SystemState__INFEASIBLE = 5
};

/// Constant 'EMERGENCY_STOP'.
/**
  * 비상 정지
 */
enum
{
  track_msgs__msg__SystemState__EMERGENCY_STOP = 6
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/SystemState in the package track_msgs.
/**
  * =============================================================
  * track_msgs/msg/SystemState.msg
  * =============================================================
  * 시스템 전체 상태 (FSM: Finite State Machine)
  * 현재 미사용, 향후 상위 시스템 통합 시 사용 예정
  *
  * 상태 전이 예시:
  *   OFFLINE → MANUAL → AUTO_STANDBY → AUTO_ACTIVE
  *   AUTO_ACTIVE → AUTO_HOLD (일시정지)
  *   AUTO_ACTIVE → INFEASIBLE (경로 추종 불가)
  *   * → EMERGENCY_STOP (비상 정지, 어디서든 전이 가능)
  *
  * 각 상태 설명:
  *   OFFLINE (0)        — 시스템 비활성 (초기화 전)
  *   MANUAL (1)         — 수동 조종 모드 (조이스틱/RC)
  *   AUTO_STANDBY (2)   — 자율주행 대기 (경로 수신 대기 중)
  *   AUTO_ACTIVE (3)    — 자율주행 활성 (경로 추종 중)
  *   AUTO_HOLD (4)      — 자율주행 일시정지 (장애물/신호 대기)
  *   INFEASIBLE (5)     — 경로 추종 불가 (감속 후 정지)
  *   EMERGENCY_STOP (6) — 비상 정지 (즉시 정지)
  * =============================================================
 */
typedef struct track_msgs__msg__SystemState
{
  /// 타임스탬프 + 좌표계 프레임
  std_msgs__msg__Header header;
  /// 현재 시스템 상태
  uint8_t state;
} track_msgs__msg__SystemState;

// Struct for a sequence of track_msgs__msg__SystemState.
typedef struct track_msgs__msg__SystemState__Sequence
{
  track_msgs__msg__SystemState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_msgs__msg__SystemState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_
