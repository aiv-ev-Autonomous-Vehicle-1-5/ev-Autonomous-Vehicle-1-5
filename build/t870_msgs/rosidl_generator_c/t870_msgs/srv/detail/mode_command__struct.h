// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from t870_msgs:srv/ModeCommand.idl
// generated code does not contain a copyright notice

#ifndef T870_MSGS__SRV__DETAIL__MODE_COMMAND__STRUCT_H_
#define T870_MSGS__SRV__DETAIL__MODE_COMMAND__STRUCT_H_

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
  t870_msgs__srv__ModeCommand_Request__GEAR_FORWARD = 0
};

/// Constant 'GEAR_NEUTRAL'.
enum
{
  t870_msgs__srv__ModeCommand_Request__GEAR_NEUTRAL = 1
};

/// Constant 'GEAR_BACKWARD'.
enum
{
  t870_msgs__srv__ModeCommand_Request__GEAR_BACKWARD = 2
};

/// Struct defined in srv/ModeCommand in the package t870_msgs.
typedef struct t870_msgs__srv__ModeCommand_Request
{
  /// Control mode (Manual / Auto)
  bool manual_mode;
  /// Emergency stop (ON / OFF)
  bool emergency_stop;
  uint8_t gear;
} t870_msgs__srv__ModeCommand_Request;

// Struct for a sequence of t870_msgs__srv__ModeCommand_Request.
typedef struct t870_msgs__srv__ModeCommand_Request__Sequence
{
  t870_msgs__srv__ModeCommand_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} t870_msgs__srv__ModeCommand_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ModeCommand in the package t870_msgs.
typedef struct t870_msgs__srv__ModeCommand_Response
{
  bool success;
} t870_msgs__srv__ModeCommand_Response;

// Struct for a sequence of t870_msgs__srv__ModeCommand_Response.
typedef struct t870_msgs__srv__ModeCommand_Response__Sequence
{
  t870_msgs__srv__ModeCommand_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} t870_msgs__srv__ModeCommand_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // T870_MSGS__SRV__DETAIL__MODE_COMMAND__STRUCT_H_
