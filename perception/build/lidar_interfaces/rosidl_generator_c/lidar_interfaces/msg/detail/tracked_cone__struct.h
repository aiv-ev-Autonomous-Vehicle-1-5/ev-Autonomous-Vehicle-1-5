// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_interfaces:msg/TrackedCone.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE__STRUCT_H_
#define LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE__STRUCT_H_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
// Member 'dimensions'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/TrackedCone in the package lidar_interfaces.
typedef struct lidar_interfaces__msg__TrackedCone
{
  std_msgs__msg__Header header;
  /// Unique Tracking ID
  uint32_t track_id;
  /// Filtered position (Kalman Filter)
  geometry_msgs__msg__Point position;
  /// Estimated velocity (vx, vy, vz)
  geometry_msgs__msg__Vector3 velocity;
  /// (width, depth, height)
  geometry_msgs__msg__Vector3 dimensions;
  float confidence;
  int32_t label;
} lidar_interfaces__msg__TrackedCone;

// Struct for a sequence of lidar_interfaces__msg__TrackedCone.
typedef struct lidar_interfaces__msg__TrackedCone__Sequence
{
  lidar_interfaces__msg__TrackedCone * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_interfaces__msg__TrackedCone__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE__STRUCT_H_
