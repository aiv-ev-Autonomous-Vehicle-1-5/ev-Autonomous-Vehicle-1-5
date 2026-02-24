// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from track_msgs:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_
#define TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OFFLINE'.
enum
{
  track_msgs__msg__SystemState__OFFLINE = 0
};

/// Constant 'MANUAL'.
enum
{
  track_msgs__msg__SystemState__MANUAL = 1
};

/// Constant 'AUTO_STANDBY'.
enum
{
  track_msgs__msg__SystemState__AUTO_STANDBY = 2
};

/// Constant 'AUTO_ACTIVE'.
enum
{
  track_msgs__msg__SystemState__AUTO_ACTIVE = 3
};

/// Constant 'AUTO_HOLD'.
enum
{
  track_msgs__msg__SystemState__AUTO_HOLD = 4
};

/// Constant 'INFEASIBLE'.
enum
{
  track_msgs__msg__SystemState__INFEASIBLE = 5
};

/// Constant 'EMERGENCY_STOP'.
enum
{
  track_msgs__msg__SystemState__EMERGENCY_STOP = 6
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/SystemState in the package track_msgs.
/**
  * track_msgs/msg/SystemState.msg
  * Global system state
 */
typedef struct track_msgs__msg__SystemState
{
  std_msgs__msg__Header header;
  uint8_t state;
} track_msgs__msg__SystemState;

// Struct for a sequence of track_msgs__msg__SystemState.
typedef struct track_msgs__msg__SystemState__Sequence
{
  track_msgs__msg__SystemState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} track_msgs__msg__SystemState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__STRUCT_H_
