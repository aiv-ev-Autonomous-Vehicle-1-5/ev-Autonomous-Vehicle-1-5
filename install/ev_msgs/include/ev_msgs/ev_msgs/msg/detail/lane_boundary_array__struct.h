// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ev_msgs:msg/LaneBoundaryArray.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__STRUCT_H_
#define EV_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__STRUCT_H_

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
#include "ev_msgs/msg/detail/lane_boundary__struct.h"

/// Struct defined in msg/LaneBoundaryArray in the package ev_msgs.
typedef struct ev_msgs__msg__LaneBoundaryArray
{
  /// 타임스탬프 + 좌표계 프레임 (base_link)
  std_msgs__msg__Header header;
  /// 차선 경계 배열 (가변 길이)
  ev_msgs__msg__LaneBoundary__Sequence boundaries;
} ev_msgs__msg__LaneBoundaryArray;

// Struct for a sequence of ev_msgs__msg__LaneBoundaryArray.
typedef struct ev_msgs__msg__LaneBoundaryArray__Sequence
{
  ev_msgs__msg__LaneBoundaryArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev_msgs__msg__LaneBoundaryArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EV_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__STRUCT_H_
