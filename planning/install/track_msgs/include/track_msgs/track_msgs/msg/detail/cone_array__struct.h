// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_msgs:msg/ConeArray.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__CONE_ARRAY__STRUCT_H_
#define TRACK_MSGS__MSG__DETAIL__CONE_ARRAY__STRUCT_H_

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
// Member 'cones'
#include "track_msgs/msg/detail/cone__struct.h"

/// Struct defined in msg/ConeArray in the package track_msgs.
/**
  * =============================================================
  * track_msgs/msg/ConeArray.msg
  * =============================================================
  * 콘 배열 — 한 프레임에서 검출된 모든 콘을 묶어서 전달
  *
  * 토픽: /perception/cones
  * 발행: Perception 노드 (LiDAR DBSCAN 클러스터링 결과)
  * 구독: LocalPlannerNode → parse_cones()에서 좌/우 분리
  *
  * 처리 흐름:
  *   Perception → ConeArray 발행
  *     → LocalPlannerNode.parse_cones()
  *       → cone.position.y >= 0 → cone_left (좌측 콘)
  *       → cone.position.y <  0 → cone_right (우측 콘)
  *     → CorridorBuilder.build() 입력으로 전달
  * =============================================================
 */
typedef struct track_msgs__msg__ConeArray
{
  /// 타임스탬프 + 좌표계 프레임 (base_link)
  std_msgs__msg__Header header;
  /// 검출된 콘 배열 (가변 길이)
  track_msgs__msg__Cone__Sequence cones;
} track_msgs__msg__ConeArray;

// Struct for a sequence of track_msgs__msg__ConeArray.
typedef struct track_msgs__msg__ConeArray__Sequence
{
  track_msgs__msg__ConeArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_msgs__msg__ConeArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_MSGS__MSG__DETAIL__CONE_ARRAY__STRUCT_H_
