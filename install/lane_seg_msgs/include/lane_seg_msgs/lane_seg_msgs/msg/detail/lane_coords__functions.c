// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lane_seg_msgs:msg/LaneCoords.idl
// generated code does not contain a copyright notice
#include "lane_seg_msgs/msg/detail/lane_coords__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `line_x`
// Member `line_y`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
lane_seg_msgs__msg__LaneCoords__init(lane_seg_msgs__msg__LaneCoords * msg)
{
  if (!msg) {
    return false;
  }
  // line_x
  if (!rosidl_runtime_c__float__Sequence__init(&msg->line_x, 0)) {
    lane_seg_msgs__msg__LaneCoords__fini(msg);
    return false;
  }
  // line_y
  if (!rosidl_runtime_c__float__Sequence__init(&msg->line_y, 0)) {
    lane_seg_msgs__msg__LaneCoords__fini(msg);
    return false;
  }
  return true;
}

void
lane_seg_msgs__msg__LaneCoords__fini(lane_seg_msgs__msg__LaneCoords * msg)
{
  if (!msg) {
    return;
  }
  // line_x
  rosidl_runtime_c__float__Sequence__fini(&msg->line_x);
  // line_y
  rosidl_runtime_c__float__Sequence__fini(&msg->line_y);
}

bool
lane_seg_msgs__msg__LaneCoords__are_equal(const lane_seg_msgs__msg__LaneCoords * lhs, const lane_seg_msgs__msg__LaneCoords * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // line_x
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->line_x), &(rhs->line_x)))
  {
    return false;
  }
  // line_y
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->line_y), &(rhs->line_y)))
  {
    return false;
  }
  return true;
}

bool
lane_seg_msgs__msg__LaneCoords__copy(
  const lane_seg_msgs__msg__LaneCoords * input,
  lane_seg_msgs__msg__LaneCoords * output)
{
  if (!input || !output) {
    return false;
  }
  // line_x
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->line_x), &(output->line_x)))
  {
    return false;
  }
  // line_y
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->line_y), &(output->line_y)))
  {
    return false;
  }
  return true;
}

lane_seg_msgs__msg__LaneCoords *
lane_seg_msgs__msg__LaneCoords__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lane_seg_msgs__msg__LaneCoords * msg = (lane_seg_msgs__msg__LaneCoords *)allocator.allocate(sizeof(lane_seg_msgs__msg__LaneCoords), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lane_seg_msgs__msg__LaneCoords));
  bool success = lane_seg_msgs__msg__LaneCoords__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lane_seg_msgs__msg__LaneCoords__destroy(lane_seg_msgs__msg__LaneCoords * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lane_seg_msgs__msg__LaneCoords__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lane_seg_msgs__msg__LaneCoords__Sequence__init(lane_seg_msgs__msg__LaneCoords__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lane_seg_msgs__msg__LaneCoords * data = NULL;

  if (size) {
    data = (lane_seg_msgs__msg__LaneCoords *)allocator.zero_allocate(size, sizeof(lane_seg_msgs__msg__LaneCoords), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lane_seg_msgs__msg__LaneCoords__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lane_seg_msgs__msg__LaneCoords__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
lane_seg_msgs__msg__LaneCoords__Sequence__fini(lane_seg_msgs__msg__LaneCoords__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      lane_seg_msgs__msg__LaneCoords__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

lane_seg_msgs__msg__LaneCoords__Sequence *
lane_seg_msgs__msg__LaneCoords__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lane_seg_msgs__msg__LaneCoords__Sequence * array = (lane_seg_msgs__msg__LaneCoords__Sequence *)allocator.allocate(sizeof(lane_seg_msgs__msg__LaneCoords__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lane_seg_msgs__msg__LaneCoords__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lane_seg_msgs__msg__LaneCoords__Sequence__destroy(lane_seg_msgs__msg__LaneCoords__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lane_seg_msgs__msg__LaneCoords__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lane_seg_msgs__msg__LaneCoords__Sequence__are_equal(const lane_seg_msgs__msg__LaneCoords__Sequence * lhs, const lane_seg_msgs__msg__LaneCoords__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lane_seg_msgs__msg__LaneCoords__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lane_seg_msgs__msg__LaneCoords__Sequence__copy(
  const lane_seg_msgs__msg__LaneCoords__Sequence * input,
  lane_seg_msgs__msg__LaneCoords__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lane_seg_msgs__msg__LaneCoords);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lane_seg_msgs__msg__LaneCoords * data =
      (lane_seg_msgs__msg__LaneCoords *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lane_seg_msgs__msg__LaneCoords__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lane_seg_msgs__msg__LaneCoords__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lane_seg_msgs__msg__LaneCoords__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
