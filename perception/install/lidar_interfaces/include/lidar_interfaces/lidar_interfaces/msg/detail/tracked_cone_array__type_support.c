// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lidar_interfaces:msg/TrackedConeArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lidar_interfaces/msg/detail/tracked_cone_array__rosidl_typesupport_introspection_c.h"
#include "lidar_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lidar_interfaces/msg/detail/tracked_cone_array__functions.h"
#include "lidar_interfaces/msg/detail/tracked_cone_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `cones`
#include "lidar_interfaces/msg/tracked_cone.h"
// Member `cones`
#include "lidar_interfaces/msg/detail/tracked_cone__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lidar_interfaces__msg__TrackedConeArray__init(message_memory);
}

void lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_fini_function(void * message_memory)
{
  lidar_interfaces__msg__TrackedConeArray__fini(message_memory);
}

size_t lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__size_function__TrackedConeArray__cones(
  const void * untyped_member)
{
  const lidar_interfaces__msg__TrackedCone__Sequence * member =
    (const lidar_interfaces__msg__TrackedCone__Sequence *)(untyped_member);
  return member->size;
}

const void * lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__get_const_function__TrackedConeArray__cones(
  const void * untyped_member, size_t index)
{
  const lidar_interfaces__msg__TrackedCone__Sequence * member =
    (const lidar_interfaces__msg__TrackedCone__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__get_function__TrackedConeArray__cones(
  void * untyped_member, size_t index)
{
  lidar_interfaces__msg__TrackedCone__Sequence * member =
    (lidar_interfaces__msg__TrackedCone__Sequence *)(untyped_member);
  return &member->data[index];
}

void lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__fetch_function__TrackedConeArray__cones(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const lidar_interfaces__msg__TrackedCone * item =
    ((const lidar_interfaces__msg__TrackedCone *)
    lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__get_const_function__TrackedConeArray__cones(untyped_member, index));
  lidar_interfaces__msg__TrackedCone * value =
    (lidar_interfaces__msg__TrackedCone *)(untyped_value);
  *value = *item;
}

void lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__assign_function__TrackedConeArray__cones(
  void * untyped_member, size_t index, const void * untyped_value)
{
  lidar_interfaces__msg__TrackedCone * item =
    ((lidar_interfaces__msg__TrackedCone *)
    lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__get_function__TrackedConeArray__cones(untyped_member, index));
  const lidar_interfaces__msg__TrackedCone * value =
    (const lidar_interfaces__msg__TrackedCone *)(untyped_value);
  *item = *value;
}

bool lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__resize_function__TrackedConeArray__cones(
  void * untyped_member, size_t size)
{
  lidar_interfaces__msg__TrackedCone__Sequence * member =
    (lidar_interfaces__msg__TrackedCone__Sequence *)(untyped_member);
  lidar_interfaces__msg__TrackedCone__Sequence__fini(member);
  return lidar_interfaces__msg__TrackedCone__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_interfaces__msg__TrackedConeArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cones",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_interfaces__msg__TrackedConeArray, cones),  // bytes offset in struct
    NULL,  // default value
    lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__size_function__TrackedConeArray__cones,  // size() function pointer
    lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__get_const_function__TrackedConeArray__cones,  // get_const(index) function pointer
    lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__get_function__TrackedConeArray__cones,  // get(index) function pointer
    lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__fetch_function__TrackedConeArray__cones,  // fetch(index, &value) function pointer
    lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__assign_function__TrackedConeArray__cones,  // assign(index, value) function pointer
    lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__resize_function__TrackedConeArray__cones  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_message_members = {
  "lidar_interfaces__msg",  // message namespace
  "TrackedConeArray",  // message name
  2,  // number of fields
  sizeof(lidar_interfaces__msg__TrackedConeArray),
  lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_message_member_array,  // message members
  lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_init_function,  // function to initialize message memory (memory has to be allocated)
  lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_message_type_support_handle = {
  0,
  &lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidar_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_interfaces, msg, TrackedConeArray)() {
  lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_interfaces, msg, TrackedCone)();
  if (!lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_message_type_support_handle.typesupport_identifier) {
    lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lidar_interfaces__msg__TrackedConeArray__rosidl_typesupport_introspection_c__TrackedConeArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
