// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_interfaces:msg/ConeArray.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_INTERFACES__MSG__DETAIL__CONE_ARRAY__STRUCT_H_
#define LIDAR_INTERFACES__MSG__DETAIL__CONE_ARRAY__STRUCT_H_

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
// Member 'cones'
#include "lidar_interfaces/msg/detail/cone__struct.h"

/// Struct defined in msg/ConeArray in the package lidar_interfaces.
typedef struct lidar_interfaces__msg__ConeArray
{
  std_msgs__msg__Header header;
  lidar_interfaces__msg__Cone__Sequence cones;
} lidar_interfaces__msg__ConeArray;

// Struct for a sequence of lidar_interfaces__msg__ConeArray.
typedef struct lidar_interfaces__msg__ConeArray__Sequence
{
  lidar_interfaces__msg__ConeArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_interfaces__msg__ConeArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_INTERFACES__MSG__DETAIL__CONE_ARRAY__STRUCT_H_
