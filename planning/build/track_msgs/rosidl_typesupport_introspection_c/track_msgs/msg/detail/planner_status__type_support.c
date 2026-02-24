// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from track_msgs:msg/PlannerStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "track_msgs/msg/detail/planner_status__rosidl_typesupport_introspection_c.h"
#include "track_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "track_msgs/msg/detail/planner_status__functions.h"
#include "track_msgs/msg/detail/planner_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  track_msgs__msg__PlannerStatus__init(message_memory);
}

void track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_fini_function(void * message_memory)
{
  track_msgs__msg__PlannerStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_msgs__msg__PlannerStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_msgs__msg__PlannerStatus, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(track_msgs__msg__PlannerStatus, reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_message_members = {
  "track_msgs__msg",  // message namespace
  "PlannerStatus",  // message name
  3,  // number of fields
  sizeof(track_msgs__msg__PlannerStatus),
  track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_message_member_array,  // message members
  track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_message_type_support_handle = {
  0,
  &track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_track_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, track_msgs, msg, PlannerStatus)() {
  track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_message_type_support_handle.typesupport_identifier) {
    track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &track_msgs__msg__PlannerStatus__rosidl_typesupport_introspection_c__PlannerStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
