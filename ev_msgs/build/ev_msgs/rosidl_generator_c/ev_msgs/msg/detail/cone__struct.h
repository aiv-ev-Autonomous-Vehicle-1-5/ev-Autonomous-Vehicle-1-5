// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ev_msgs:msg/Cone.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__CONE__STRUCT_H_
#define EV_MSGS__MSG__DETAIL__CONE__STRUCT_H_

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

/// Struct defined in msg/Cone in the package ev_msgs.
typedef struct ev_msgs__msg__Cone
{
  /// 클러스터 무게중심 (x, y, z)
  geometry_msgs__msg__Point position;
  /// 원기둥 반지름
  float radius;
  /// 원기둥 높이
  float height;
  /// 시그모이드 정규화 신뢰도 (0.0~1.0)
  float confidence;
  /// 클러스터 ID
  int32_t label;
} ev_msgs__msg__Cone;

// Struct for a sequence of ev_msgs__msg__Cone.
typedef struct ev_msgs__msg__Cone__Sequence
{
  ev_msgs__msg__Cone * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev_msgs__msg__Cone__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EV_MSGS__MSG__DETAIL__CONE__STRUCT_H_
