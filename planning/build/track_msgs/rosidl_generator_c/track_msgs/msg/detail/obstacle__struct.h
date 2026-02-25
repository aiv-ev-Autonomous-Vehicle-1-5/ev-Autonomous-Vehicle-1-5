// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_
#define TRACK_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TYPE_STATIC'.
/**
  * 장애물 유형 상수
  * 정적 장애물 (PE 드럼, 고정 장벽 등)
 */
enum
{
  track_msgs__msg__Obstacle__TYPE_STATIC = 0
};

/// Constant 'TYPE_DYNAMIC'.
/**
  * 동적 장애물 (보행자, 다른 차량 등)
 */
enum
{
  track_msgs__msg__Obstacle__TYPE_DYNAMIC = 1
};

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'dimensions'
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Obstacle in the package track_msgs.
/**
  * =============================================================
  * track_msgs/msg/Obstacle.msg
  * =============================================================
  * 단일 장애물 (정적 또는 동적) — 현재 미사용, 향후 확장용
  *
  * 용도 (예정):
  *   - PE 드럼, 보행자 등 장애물의 위치/크기/속도 정보
  *   - 장애물 회피 플래너에서 사용 예정
  *
  * 대회 규격 참고:
  *   - PE 드럼: 직경 500mm, 높이 840mm
  *   - 교통 콘: 현재 Cone.msg로 별도 처리
  * =============================================================
 */
typedef struct track_msgs__msg__Obstacle
{
  /// 장애물 중심 좌표 (x, y, z)
  geometry_msgs__msg__Point position;
  /// 장애물 크기 (width, depth, height)
  geometry_msgs__msg__Vector3 dimensions;
  /// 추정 속도 (vx, vy, vz) — 동적 장애물용
  geometry_msgs__msg__Vector3 velocity;
  /// 이 장애물의 유형
  uint8_t type;
} track_msgs__msg__Obstacle;

// Struct for a sequence of track_msgs__msg__Obstacle.
typedef struct track_msgs__msg__Obstacle__Sequence
{
  track_msgs__msg__Obstacle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_msgs__msg__Obstacle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_MSGS__MSG__DETAIL__OBSTACLE__STRUCT_H_
