// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_
#define TRACK_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_

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
// Member 'obstacles'
#include "track_msgs/msg/detail/obstacle__struct.h"

/// Struct defined in msg/ObstacleArray in the package track_msgs.
/**
  * =============================================================
  * track_msgs/msg/ObstacleArray.msg
  * =============================================================
  * 장애물 배열 — 현재 미사용, 향후 장애물 회피 기능 확장 시 사용 예정
  *
  * 토픽 (예정): /perception/obstacles
  * 발행 (예정): Perception 노드
  * 구독 (예정): 장애물 회피 플래너
  * =============================================================
 */
typedef struct track_msgs__msg__ObstacleArray
{
  /// 타임스탬프 + 좌표계 프레임 (base_link)
  std_msgs__msg__Header header;
  /// 장애물 배열 (가변 길이)
  track_msgs__msg__Obstacle__Sequence obstacles;
} track_msgs__msg__ObstacleArray;

// Struct for a sequence of track_msgs__msg__ObstacleArray.
typedef struct track_msgs__msg__ObstacleArray__Sequence
{
  track_msgs__msg__ObstacleArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_msgs__msg__ObstacleArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_
