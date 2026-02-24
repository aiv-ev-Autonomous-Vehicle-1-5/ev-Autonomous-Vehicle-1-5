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
  * track_msgs/msg/LaneBoundaryArray.msg
 */
typedef struct track_msgs__msg__LaneBoundaryArray
{
  std_msgs__msg__Header header;
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
