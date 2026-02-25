// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_msgs:msg/LaneBoundaryArray.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__STRUCT_H_
#define TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'boundaries'
#include "track_msgs/msg/detail/lane_boundary__struct.h"

/// Struct defined in msg/LaneBoundaryArray in the package track_msgs.
/**
  * =============================================================
  * track_msgs/msg/LaneBoundaryArray.msg
  * =============================================================
  * 차선 경계 배열 — 한 프레임에서 인식된 모든 차선 경계를 묶어서 전달
  *
  * 토픽: /perception/lane_boundaries
  * 발행: Perception 노드 (카메라 차선 인식 결과)
  * 구독: LocalPlannerNode → parse_lanes()에서 좌/우 분리
  *
  * 처리 흐름:
  *   Perception → LaneBoundaryArray 발행
  *     → LocalPlannerNode.parse_lanes()
  *       → boundary.side == LEFT  → lane_left (좌측 차선 경계점)
  *       → boundary.side == RIGHT → lane_right (우측 차선 경계점)
  *     → CorridorBuilder.build() 입력으로 전달
  *
  * 참고:
  *   - boundaries 배열에는 보통 2개(좌1, 우1)의 경계가 포함
  *   - 한쪽 차선만 인식된 경우 1개만 포함될 수 있음
  * =============================================================
 */
typedef struct track_msgs__msg__LaneBoundaryArray
{
  /// 타임스탬프 + 좌표계 프레임 (base_link)
  std_msgs__msg__Header header;
  /// 차선 경계 배열 (가변 길이)
  track_msgs__msg__LaneBoundary__Sequence boundaries;
} track_msgs__msg__LaneBoundaryArray;

// Struct for a sequence of track_msgs__msg__LaneBoundaryArray.
typedef struct track_msgs__msg__LaneBoundaryArray__Sequence
{
  track_msgs__msg__LaneBoundaryArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_msgs__msg__LaneBoundaryArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__STRUCT_H_
