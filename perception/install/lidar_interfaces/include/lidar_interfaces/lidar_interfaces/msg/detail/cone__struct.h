// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_interfaces:msg/Cone.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_INTERFACES__MSG__DETAIL__CONE__STRUCT_H_
#define LIDAR_INTERFACES__MSG__DETAIL__CONE__STRUCT_H_

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
// Member 'dimensions'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Cone in the package lidar_interfaces.
typedef struct lidar_interfaces__msg__Cone
{
  std_msgs__msg__Header header;
  /// Centroid (x, y, z)
  geometry_msgs__msg__Point position;
  /// (width, depth, height)
  geometry_msgs__msg__Vector3 dimensions;
  /// Detection confidence (0.0 to 1.0)
  float confidence;
  /// Cluster label or class ID
  int32_t label;
} lidar_interfaces__msg__Cone;

// Struct for a sequence of lidar_interfaces__msg__Cone.
typedef struct lidar_interfaces__msg__Cone__Sequence
{
  lidar_interfaces__msg__Cone * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_interfaces__msg__Cone__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_INTERFACES__MSG__DETAIL__CONE__STRUCT_H_
