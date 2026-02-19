// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lidar_interfaces:msg/ConeArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lidar_interfaces/msg/detail/cone_array__rosidl_typesupport_introspection_c.h"
#include "lidar_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lidar_interfaces/msg/detail/cone_array__functions.h"
#include "lidar_interfaces/msg/detail/cone_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `cones`
#include "lidar_interfaces/msg/cone.h"
// Member `cones`
#include "lidar_interfaces/msg/detail/cone__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lidar_interfaces__msg__ConeArray__init(message_memory);
}

void lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_fini_function(void * message_memory)
{
  lidar_interfaces__msg__ConeArray__fini(message_memory);
}

size_t lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__size_function__ConeArray__cones(
  const void * untyped_member)
{
  const lidar_interfaces__msg__Cone__Sequence * member =
    (const lidar_interfaces__msg__Cone__Sequence *)(untyped_member);
  return member->size;
}

const void * lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__get_const_function__ConeArray__cones(
  const void * untyped_member, size_t index)
{
  const lidar_interfaces__msg__Cone__Sequence * member =
    (const lidar_interfaces__msg__Cone__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__get_function__ConeArray__cones(
  void * untyped_member, size_t index)
{
  lidar_interfaces__msg__Cone__Sequence * member =
    (lidar_interfaces__msg__Cone__Sequence *)(untyped_member);
  return &member->data[index];
}

void lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__fetch_function__ConeArray__cones(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const lidar_interfaces__msg__Cone * item =
    ((const lidar_interfaces__msg__Cone *)
    lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__get_const_function__ConeArray__cones(untyped_member, index));
  lidar_interfaces__msg__Cone * value =
    (lidar_interfaces__msg__Cone *)(untyped_value);
  *value = *item;
}

void lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__assign_function__ConeArray__cones(
  void * untyped_member, size_t index, const void * untyped_value)
{
  lidar_interfaces__msg__Cone * item =
    ((lidar_interfaces__msg__Cone *)
    lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__get_function__ConeArray__cones(untyped_member, index));
  const lidar_interfaces__msg__Cone * value =
    (const lidar_interfaces__msg__Cone *)(untyped_value);
  *item = *value;
}

bool lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__resize_function__ConeArray__cones(
  void * untyped_member, size_t size)
{
  lidar_interfaces__msg__Cone__Sequence * member =
    (lidar_interfaces__msg__Cone__Sequence *)(untyped_member);
  lidar_interfaces__msg__Cone__Sequence__fini(member);
  return lidar_interfaces__msg__Cone__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_interfaces__msg__ConeArray, header),  // bytes offset in struct
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
    offsetof(lidar_interfaces__msg__ConeArray, cones),  // bytes offset in struct
    NULL,  // default value
    lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__size_function__ConeArray__cones,  // size() function pointer
    lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__get_const_function__ConeArray__cones,  // get_const(index) function pointer
    lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__get_function__ConeArray__cones,  // get(index) function pointer
    lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__fetch_function__ConeArray__cones,  // fetch(index, &value) function pointer
    lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__assign_function__ConeArray__cones,  // assign(index, value) function pointer
    lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__resize_function__ConeArray__cones  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_message_members = {
  "lidar_interfaces__msg",  // message namespace
  "ConeArray",  // message name
  2,  // number of fields
  sizeof(lidar_interfaces__msg__ConeArray),
  lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_message_member_array,  // message members
  lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_init_function,  // function to initialize message memory (memory has to be allocated)
  lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_message_type_support_handle = {
  0,
  &lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidar_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_interfaces, msg, ConeArray)() {
  lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_interfaces, msg, Cone)();
  if (!lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_message_type_support_handle.typesupport_identifier) {
    lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lidar_interfaces__msg__ConeArray__rosidl_typesupport_introspection_c__ConeArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
