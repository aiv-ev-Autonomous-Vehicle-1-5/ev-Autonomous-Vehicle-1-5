// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ev_msgs:msg/LaneBoundary.idl
// generated code does not contain a copyright notice
#include "ev_msgs/msg/detail/lane_boundary__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `points`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
ev_msgs__msg__LaneBoundary__init(ev_msgs__msg__LaneBoundary * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ev_msgs__msg__LaneBoundary__fini(msg);
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->points, 0)) {
    ev_msgs__msg__LaneBoundary__fini(msg);
    return false;
  }
  // confidence
  return true;
}

void
ev_msgs__msg__LaneBoundary__fini(ev_msgs__msg__LaneBoundary * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // points
  geometry_msgs__msg__Point__Sequence__fini(&msg->points);
  // confidence
}

bool
ev_msgs__msg__LaneBoundary__are_equal(const ev_msgs__msg__LaneBoundary * lhs, const ev_msgs__msg__LaneBoundary * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->points), &(rhs->points)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  return true;
}

bool
ev_msgs__msg__LaneBoundary__copy(
  const ev_msgs__msg__LaneBoundary * input,
  ev_msgs__msg__LaneBoundary * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->points), &(output->points)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  return true;
}

ev_msgs__msg__LaneBoundary *
ev_msgs__msg__LaneBoundary__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ev_msgs__msg__LaneBoundary * msg = (ev_msgs__msg__LaneBoundary *)allocator.allocate(sizeof(ev_msgs__msg__LaneBoundary), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ev_msgs__msg__LaneBoundary));
  bool success = ev_msgs__msg__LaneBoundary__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ev_msgs__msg__LaneBoundary__destroy(ev_msgs__msg__LaneBoundary * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ev_msgs__msg__LaneBoundary__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ev_msgs__msg__LaneBoundary__Sequence__init(ev_msgs__msg__LaneBoundary__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ev_msgs__msg__LaneBoundary * data = NULL;

  if (size) {
    data = (ev_msgs__msg__LaneBoundary *)allocator.zero_allocate(size, sizeof(ev_msgs__msg__LaneBoundary), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ev_msgs__msg__LaneBoundary__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ev_msgs__msg__LaneBoundary__fini(&data[i - 1]);
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
ev_msgs__msg__LaneBoundary__Sequence__fini(ev_msgs__msg__LaneBoundary__Sequence * array)
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
      ev_msgs__msg__LaneBoundary__fini(&array->data[i]);
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

ev_msgs__msg__LaneBoundary__Sequence *
ev_msgs__msg__LaneBoundary__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ev_msgs__msg__LaneBoundary__Sequence * array = (ev_msgs__msg__LaneBoundary__Sequence *)allocator.allocate(sizeof(ev_msgs__msg__LaneBoundary__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ev_msgs__msg__LaneBoundary__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ev_msgs__msg__LaneBoundary__Sequence__destroy(ev_msgs__msg__LaneBoundary__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ev_msgs__msg__LaneBoundary__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ev_msgs__msg__LaneBoundary__Sequence__are_equal(const ev_msgs__msg__LaneBoundary__Sequence * lhs, const ev_msgs__msg__LaneBoundary__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ev_msgs__msg__LaneBoundary__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ev_msgs__msg__LaneBoundary__Sequence__copy(
  const ev_msgs__msg__LaneBoundary__Sequence * input,
  ev_msgs__msg__LaneBoundary__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ev_msgs__msg__LaneBoundary);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ev_msgs__msg__LaneBoundary * data =
      (ev_msgs__msg__LaneBoundary *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ev_msgs__msg__LaneBoundary__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ev_msgs__msg__LaneBoundary__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ev_msgs__msg__LaneBoundary__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
