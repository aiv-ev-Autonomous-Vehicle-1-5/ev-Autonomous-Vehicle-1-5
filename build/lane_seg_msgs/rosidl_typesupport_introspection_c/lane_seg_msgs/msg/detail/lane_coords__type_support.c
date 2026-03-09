// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lane_seg_msgs:msg/LaneCoords.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lane_seg_msgs/msg/detail/lane_coords__rosidl_typesupport_introspection_c.h"
#include "lane_seg_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lane_seg_msgs/msg/detail/lane_coords__functions.h"
#include "lane_seg_msgs/msg/detail/lane_coords__struct.h"


// Include directives for member types
// Member `line_x`
// Member `line_y`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lane_seg_msgs__msg__LaneCoords__init(message_memory);
}

void lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_fini_function(void * message_memory)
{
  lane_seg_msgs__msg__LaneCoords__fini(message_memory);
}

size_t lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__size_function__LaneCoords__line_x(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_const_function__LaneCoords__line_x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_function__LaneCoords__line_x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__fetch_function__LaneCoords__line_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_const_function__LaneCoords__line_x(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__assign_function__LaneCoords__line_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_function__LaneCoords__line_x(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__resize_function__LaneCoords__line_x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__size_function__LaneCoords__line_y(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_const_function__LaneCoords__line_y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_function__LaneCoords__line_y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__fetch_function__LaneCoords__line_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_const_function__LaneCoords__line_y(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__assign_function__LaneCoords__line_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_function__LaneCoords__line_y(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__resize_function__LaneCoords__line_y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_message_member_array[2] = {
  {
    "line_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lane_seg_msgs__msg__LaneCoords, line_x),  // bytes offset in struct
    NULL,  // default value
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__size_function__LaneCoords__line_x,  // size() function pointer
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_const_function__LaneCoords__line_x,  // get_const(index) function pointer
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_function__LaneCoords__line_x,  // get(index) function pointer
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__fetch_function__LaneCoords__line_x,  // fetch(index, &value) function pointer
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__assign_function__LaneCoords__line_x,  // assign(index, value) function pointer
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__resize_function__LaneCoords__line_x  // resize(index) function pointer
  },
  {
    "line_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lane_seg_msgs__msg__LaneCoords, line_y),  // bytes offset in struct
    NULL,  // default value
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__size_function__LaneCoords__line_y,  // size() function pointer
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_const_function__LaneCoords__line_y,  // get_const(index) function pointer
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__get_function__LaneCoords__line_y,  // get(index) function pointer
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__fetch_function__LaneCoords__line_y,  // fetch(index, &value) function pointer
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__assign_function__LaneCoords__line_y,  // assign(index, value) function pointer
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__resize_function__LaneCoords__line_y  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_message_members = {
  "lane_seg_msgs__msg",  // message namespace
  "LaneCoords",  // message name
  2,  // number of fields
  sizeof(lane_seg_msgs__msg__LaneCoords),
  lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_message_member_array,  // message members
  lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_init_function,  // function to initialize message memory (memory has to be allocated)
  lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_message_type_support_handle = {
  0,
  &lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lane_seg_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lane_seg_msgs, msg, LaneCoords)() {
  if (!lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_message_type_support_handle.typesupport_identifier) {
    lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lane_seg_msgs__msg__LaneCoords__rosidl_typesupport_introspection_c__LaneCoords_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
