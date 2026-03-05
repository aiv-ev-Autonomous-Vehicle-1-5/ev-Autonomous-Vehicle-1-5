// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from t870_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef T870_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_
#define T870_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GEAR_FORWARD'.
/**
  * Gear (Forward / Neutral / Backward)
 */
enum
{
  t870_msgs__msg__Feedback__GEAR_FORWARD = 0
};

/// Constant 'GEAR_NEUTRAL'.
enum
{
  t870_msgs__msg__Feedback__GEAR_NEUTRAL = 1
};

/// Constant 'GEAR_BACKWARD'.
enum
{
  t870_msgs__msg__Feedback__GEAR_BACKWARD = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Feedback in the package t870_msgs.
/**
  * t870_msgs/msg/Feedback.msg
 */
typedef struct t870_msgs__msg__Feedback
{
  /// Message header
  std_msgs__msg__Header header;
  /// Control mode (Manual / Auto)
  bool manual_mode;
  /// Emergency stop (ON / OFF)
  bool emergency_stop;
  uint8_t gear;
  /// Vehicle speed (m/s)
  double speed;
  /// Steering angle (rad)
  double steering;
  /// Health check counter (0 - 255)
  uint8_t heartbeat;
} t870_msgs__msg__Feedback;

// Struct for a sequence of t870_msgs__msg__Feedback.
typedef struct t870_msgs__msg__Feedback__Sequence
{
  t870_msgs__msg__Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} t870_msgs__msg__Feedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // T870_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_
