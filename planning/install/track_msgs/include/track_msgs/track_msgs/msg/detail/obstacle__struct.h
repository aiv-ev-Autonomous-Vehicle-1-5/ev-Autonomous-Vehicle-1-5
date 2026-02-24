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
enum
{
  track_msgs__msg__Obstacle__TYPE_STATIC = 0
};

/// Constant 'TYPE_DYNAMIC'.
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
  * track_msgs/msg/Obstacle.msg
  * Single obstacle (static or dynamic)
 */
typedef struct track_msgs__msg__Obstacle
{
  /// Centroid (x, y, z)
  geometry_msgs__msg__Point position;
  /// (width, depth, height)
  geometry_msgs__msg__Vector3 dimensions;
  /// Estimated velocity (vx, vy, vz) for dynamic
  geometry_msgs__msg__Vector3 velocity;
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
