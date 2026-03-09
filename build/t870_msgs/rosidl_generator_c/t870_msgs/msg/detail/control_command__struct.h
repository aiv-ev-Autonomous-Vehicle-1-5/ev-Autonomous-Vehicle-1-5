// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from t870_msgs:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef T870_MSGS__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_
#define T870_MSGS__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ControlCommand in the package t870_msgs.
/**
  * t870_msgs/msg/ControlCommand.msg
 */
typedef struct t870_msgs__msg__ControlCommand
{
  /// Vehicle speed (m/s)
  double speed;
  /// Steering angle (rad)
  double steering;
} t870_msgs__msg__ControlCommand;

// Struct for a sequence of t870_msgs__msg__ControlCommand.
typedef struct t870_msgs__msg__ControlCommand__Sequence
{
  t870_msgs__msg__ControlCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} t870_msgs__msg__ControlCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // T870_MSGS__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_
