// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ev_msgs:msg/LaneBoundary.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_H_
#define EV_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_H_

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
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/LaneBoundary in the package ev_msgs.
typedef struct ev_msgs__msg__LaneBoundary
{
  /// 타임스탬프 + 좌표계 프레임 (base_link)
  std_msgs__msg__Header header;
  /// 순서 정렬된 경계점 배열 — 차량에서 가까운 점부터
  geometry_msgs__msg__Point__Sequence points;
  /// 검출 신뢰도 (0.0 ~ 1.0)
  float confidence;
} ev_msgs__msg__LaneBoundary;

// Struct for a sequence of ev_msgs__msg__LaneBoundary.
typedef struct ev_msgs__msg__LaneBoundary__Sequence
{
  ev_msgs__msg__LaneBoundary * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev_msgs__msg__LaneBoundary__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EV_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_H_
