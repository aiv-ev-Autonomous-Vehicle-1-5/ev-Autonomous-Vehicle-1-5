// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from erp42_msgs:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef ERP42_MSGS__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_
#define ERP42_MSGS__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ControlCommand in the package erp42_msgs.
/**
  * erp42_msgs/msg/ControlCommand.msg
 */
typedef struct erp42_msgs__msg__ControlCommand
{
  /// Vehicle speed (m/s)
  double speed;
  /// Steering angle (rad)
  double steering;
  /// Brake
  /// ERP42 (0 - 150)
  uint8_t brake;
} erp42_msgs__msg__ControlCommand;

// Struct for a sequence of erp42_msgs__msg__ControlCommand.
typedef struct erp42_msgs__msg__ControlCommand__Sequence
{
  erp42_msgs__msg__ControlCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} erp42_msgs__msg__ControlCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ERP42_MSGS__MSG__DETAIL__CONTROL_COMMAND__STRUCT_H_
