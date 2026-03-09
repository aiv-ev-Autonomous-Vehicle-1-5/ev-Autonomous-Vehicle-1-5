// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ev_msgs:msg/BBox.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__B_BOX__STRUCT_H_
#define EV_MSGS__MSG__DETAIL__B_BOX__STRUCT_H_

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

/// Struct defined in msg/BBox in the package ev_msgs.
typedef struct ev_msgs__msg__BBox
{
  /// 바운딩박스 중심 (x, y, z)
  geometry_msgs__msg__Point position;
  /// 바운딩박스 X 크기
  float size_x;
  /// 바운딩박스 Y 크기
  float size_y;
  /// 바운딩박스 Z 크기 (높이)
  float size_z;
  /// 시그모이드 정규화 신뢰도 (0.0~1.0)
  float confidence;
  /// 클러스터 ID
  int32_t label;
} ev_msgs__msg__BBox;

// Struct for a sequence of ev_msgs__msg__BBox.
typedef struct ev_msgs__msg__BBox__Sequence
{
  ev_msgs__msg__BBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev_msgs__msg__BBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EV_MSGS__MSG__DETAIL__B_BOX__STRUCT_H_
