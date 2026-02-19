// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lidar_interfaces:msg/Cone.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lidar_interfaces/msg/detail/cone__rosidl_typesupport_introspection_c.h"
#include "lidar_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lidar_interfaces/msg/detail/cone__functions.h"
#include "lidar_interfaces/msg/detail/cone__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `position`
#include "geometry_msgs/msg/point.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `dimensions`
#include "geometry_msgs/msg/vector3.h"
// Member `dimensions`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lidar_interfaces__msg__Cone__init(message_memory);
}

void lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_fini_function(void * message_memory)
{
  lidar_interfaces__msg__Cone__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_interfaces__msg__Cone, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_interfaces__msg__Cone, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dimensions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_interfaces__msg__Cone, dimensions),  // bytes offset in struct
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
    offsetof(lidar_interfaces__msg__Cone, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "label",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_interfaces__msg__Cone, label),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_message_members = {
  "lidar_interfaces__msg",  // message namespace
  "Cone",  // message name
  5,  // number of fields
  sizeof(lidar_interfaces__msg__Cone),
  lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_message_member_array,  // message members
  lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_init_function,  // function to initialize message memory (memory has to be allocated)
  lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_message_type_support_handle = {
  0,
  &lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidar_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_interfaces, msg, Cone)() {
  lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_message_type_support_handle.typesupport_identifier) {
    lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lidar_interfaces__msg__Cone__rosidl_typesupport_introspection_c__Cone_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
