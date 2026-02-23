// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_msgs:msg/PlannerStatus.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__STRUCT_H_
#define TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OK'.
enum
{
  track_msgs__msg__PlannerStatus__OK = 0
};

/// Constant 'STOP'.
enum
{
  track_msgs__msg__PlannerStatus__STOP = 1
};

/// Constant 'INFEASIBLE'.
enum
{
  track_msgs__msg__PlannerStatus__INFEASIBLE = 2
};

/// Constant 'STALE'.
enum
{
  track_msgs__msg__PlannerStatus__STALE = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/PlannerStatus in the package track_msgs.
/**
  * Planner status codes
 */
typedef struct track_msgs__msg__PlannerStatus
{
  std_msgs__msg__Header header;
  uint8_t status;
  rosidl_runtime_c__String reason;
} track_msgs__msg__PlannerStatus;

// Struct for a sequence of track_msgs__msg__PlannerStatus.
typedef struct track_msgs__msg__PlannerStatus__Sequence
{
  track_msgs__msg__PlannerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_msgs__msg__PlannerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__STRUCT_H_
