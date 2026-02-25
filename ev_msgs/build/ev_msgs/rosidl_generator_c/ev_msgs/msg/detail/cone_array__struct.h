// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ev_msgs:msg/ConeArray.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__CONE_ARRAY__STRUCT_H_
#define EV_MSGS__MSG__DETAIL__CONE_ARRAY__STRUCT_H_

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
#include "ev_msgs/msg/detail/cone__struct.h"

/// Struct defined in msg/ConeArray in the package ev_msgs.
typedef struct ev_msgs__msg__ConeArray
{
  std_msgs__msg__Header header;
  ev_msgs__msg__Cone__Sequence cones;
} ev_msgs__msg__ConeArray;

// Struct for a sequence of ev_msgs__msg__ConeArray.
typedef struct ev_msgs__msg__ConeArray__Sequence
{
  ev_msgs__msg__ConeArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev_msgs__msg__ConeArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EV_MSGS__MSG__DETAIL__CONE_ARRAY__STRUCT_H_
