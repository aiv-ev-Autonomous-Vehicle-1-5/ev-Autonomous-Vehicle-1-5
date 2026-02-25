// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ev_msgs:msg/LaneBoundary.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ev_msgs/msg/detail/lane_boundary__rosidl_typesupport_introspection_c.h"
#include "ev_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ev_msgs/msg/detail/lane_boundary__functions.h"
#include "ev_msgs/msg/detail/lane_boundary__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `points`
#include "geometry_msgs/msg/point.h"
// Member `points`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ev_msgs__msg__LaneBoundary__init(message_memory);
}

void ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_fini_function(void * message_memory)
{
  ev_msgs__msg__LaneBoundary__fini(message_memory);
}

size_t ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__size_function__LaneBoundary__points(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__get_const_function__LaneBoundary__points(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__get_function__LaneBoundary__points(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__fetch_function__LaneBoundary__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__get_const_function__LaneBoundary__points(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__assign_function__LaneBoundary__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__get_function__LaneBoundary__points(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__resize_function__LaneBoundary__points(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ev_msgs__msg__LaneBoundary, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ev_msgs__msg__LaneBoundary, points),  // bytes offset in struct
    NULL,  // default value
    ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__size_function__LaneBoundary__points,  // size() function pointer
    ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__get_const_function__LaneBoundary__points,  // get_const(index) function pointer
    ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__get_function__LaneBoundary__points,  // get(index) function pointer
    ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__fetch_function__LaneBoundary__points,  // fetch(index, &value) function pointer
    ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__assign_function__LaneBoundary__points,  // assign(index, value) function pointer
    ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__resize_function__LaneBoundary__points  // resize(index) function pointer
  },
  {
    "side",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ev_msgs__msg__LaneBoundary, side),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ev_msgs__msg__LaneBoundary, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_message_members = {
  "ev_msgs__msg",  // message namespace
  "LaneBoundary",  // message name
  4,  // number of fields
  sizeof(ev_msgs__msg__LaneBoundary),
  ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_message_member_array,  // message members
  ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_init_function,  // function to initialize message memory (memory has to be allocated)
  ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_message_type_support_handle = {
  0,
  &ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ev_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ev_msgs, msg, LaneBoundary)() {
  ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_message_type_support_handle.typesupport_identifier) {
    ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ev_msgs__msg__LaneBoundary__rosidl_typesupport_introspection_c__LaneBoundary_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
