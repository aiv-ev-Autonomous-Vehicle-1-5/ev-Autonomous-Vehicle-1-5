// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from erp42_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef ERP42_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_
#define ERP42_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GEAR_DRIVE'.
/**
  * Gear (Drive / Neutral / Reverse)
 */
enum
{
  erp42_msgs__msg__Feedback__GEAR_DRIVE = 0
};

/// Constant 'GEAR_NEUTRAL'.
enum
{
  erp42_msgs__msg__Feedback__GEAR_NEUTRAL = 1
};

/// Constant 'GEAR_REVERSE'.
enum
{
  erp42_msgs__msg__Feedback__GEAR_REVERSE = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Feedback in the package erp42_msgs.
/**
  * erp42_msgs/msg/Feedback.msg
 */
typedef struct erp42_msgs__msg__Feedback
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
  /// Braking
  /// ERP42 (0 - 150)
  uint8_t brake;
  /// Encoder count (-2^31 - 2^31)
  int32_t encoder_count;
  /// Health check counter (0 - 255)
  uint8_t heartbeat;
} erp42_msgs__msg__Feedback;

// Struct for a sequence of erp42_msgs__msg__Feedback.
typedef struct erp42_msgs__msg__Feedback__Sequence
{
  erp42_msgs__msg__Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} erp42_msgs__msg__Feedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ERP42_MSGS__MSG__DETAIL__FEEDBACK__STRUCT_H_
