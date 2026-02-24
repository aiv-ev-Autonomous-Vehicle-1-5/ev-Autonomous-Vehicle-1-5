// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_msgs:msg/LaneBoundary.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_H_
#define TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LEFT'.
enum
{
  track_msgs__msg__LaneBoundary__LEFT = 0
};

/// Constant 'RIGHT'.
enum
{
  track_msgs__msg__LaneBoundary__RIGHT = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/LaneBoundary in the package track_msgs.
/**
  * track_msgs/msg/LaneBoundary.msg
  * Single lane boundary as a sequence of points
 */
typedef struct track_msgs__msg__LaneBoundary
{
  std_msgs__msg__Header header;
  /// Ordered boundary points
  geometry_msgs__msg__Point__Sequence points;
  /// LEFT or RIGHT
  uint8_t side;
  /// Detection confidence (0.0 to 1.0)
  float confidence;
} track_msgs__msg__LaneBoundary;

// Struct for a sequence of track_msgs__msg__LaneBoundary.
typedef struct track_msgs__msg__LaneBoundary__Sequence
{
  track_msgs__msg__LaneBoundary * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_msgs__msg__LaneBoundary__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_H_
