// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lane_seg_msgs:msg/LaneCoords.idl
// generated code does not contain a copyright notice

#ifndef LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__STRUCT_H_
#define LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'line_x'
// Member 'line_y'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/LaneCoords in the package lane_seg_msgs.
typedef struct lane_seg_msgs__msg__LaneCoords
{
  rosidl_runtime_c__float__Sequence line_x;
  rosidl_runtime_c__float__Sequence line_y;
} lane_seg_msgs__msg__LaneCoords;

// Struct for a sequence of lane_seg_msgs__msg__LaneCoords.
typedef struct lane_seg_msgs__msg__LaneCoords__Sequence
{
  lane_seg_msgs__msg__LaneCoords * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lane_seg_msgs__msg__LaneCoords__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__STRUCT_H_
