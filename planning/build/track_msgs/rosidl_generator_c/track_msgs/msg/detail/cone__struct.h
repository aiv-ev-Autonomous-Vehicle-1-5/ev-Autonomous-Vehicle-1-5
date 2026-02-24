// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_msgs:msg/Cone.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__CONE__STRUCT_H_
#define TRACK_MSGS__MSG__DETAIL__CONE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'dimensions'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Cone in the package track_msgs.
/**
  * track_msgs/msg/Cone.msg
  * Single detected cone (lavacone / traffic cone)
 */
typedef struct track_msgs__msg__Cone
{
  /// Centroid (x, y, z)
  geometry_msgs__msg__Point position;
  /// (width, depth, height)
  geometry_msgs__msg__Vector3 dimensions;
  /// Detection confidence (0.0 to 1.0)
  float confidence;
  /// Cluster label or class ID
  int32_t label;
} track_msgs__msg__Cone;

// Struct for a sequence of track_msgs__msg__Cone.
typedef struct track_msgs__msg__Cone__Sequence
{
  track_msgs__msg__Cone * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_msgs__msg__Cone__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_MSGS__MSG__DETAIL__CONE__STRUCT_H_
