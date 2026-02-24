// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from track_msgs:msg/LaneBoundaryArray.idl
// generated code does not contain a copyright notice
#include "track_msgs/msg/detail/lane_boundary_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `boundaries`
#include "track_msgs/msg/detail/lane_boundary__functions.h"

bool
track_msgs__msg__LaneBoundaryArray__init(track_msgs__msg__LaneBoundaryArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    track_msgs__msg__LaneBoundaryArray__fini(msg);
    return false;
  }
  // boundaries
  if (!track_msgs__msg__LaneBoundary__Sequence__init(&msg->boundaries, 0)) {
    track_msgs__msg__LaneBoundaryArray__fini(msg);
    return false;
  }
  return true;
}

void
track_msgs__msg__LaneBoundaryArray__fini(track_msgs__msg__LaneBoundaryArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // boundaries
  track_msgs__msg__LaneBoundary__Sequence__fini(&msg->boundaries);
}

bool
track_msgs__msg__LaneBoundaryArray__are_equal(const track_msgs__msg__LaneBoundaryArray * lhs, const track_msgs__msg__LaneBoundaryArray * rhs)
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
  // boundaries
  if (!track_msgs__msg__LaneBoundary__Sequence__are_equal(
      &(lhs->boundaries), &(rhs->boundaries)))
  {
    return false;
  }
  return true;
}

bool
track_msgs__msg__LaneBoundaryArray__copy(
  const track_msgs__msg__LaneBoundaryArray * input,
  track_msgs__msg__LaneBoundaryArray * output)
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
  // boundaries
  if (!track_msgs__msg__LaneBoundary__Sequence__copy(
      &(input->boundaries), &(output->boundaries)))
  {
    return false;
  }
  return true;
}

track_msgs__msg__LaneBoundaryArray *
track_msgs__msg__LaneBoundaryArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  track_msgs__msg__LaneBoundaryArray * msg = (track_msgs__msg__LaneBoundaryArray *)allocator.allocate(sizeof(track_msgs__msg__LaneBoundaryArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(track_msgs__msg__LaneBoundaryArray));
  bool success = track_msgs__msg__LaneBoundaryArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
track_msgs__msg__LaneBoundaryArray__destroy(track_msgs__msg__LaneBoundaryArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    track_msgs__msg__LaneBoundaryArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
track_msgs__msg__LaneBoundaryArray__Sequence__init(track_msgs__msg__LaneBoundaryArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  track_msgs__msg__LaneBoundaryArray * data = NULL;

  if (size) {
    data = (track_msgs__msg__LaneBoundaryArray *)allocator.zero_allocate(size, sizeof(track_msgs__msg__LaneBoundaryArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = track_msgs__msg__LaneBoundaryArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        track_msgs__msg__LaneBoundaryArray__fini(&data[i - 1]);
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
track_msgs__msg__LaneBoundaryArray__Sequence__fini(track_msgs__msg__LaneBoundaryArray__Sequence * array)
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
      track_msgs__msg__LaneBoundaryArray__fini(&array->data[i]);
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

track_msgs__msg__LaneBoundaryArray__Sequence *
track_msgs__msg__LaneBoundaryArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  track_msgs__msg__LaneBoundaryArray__Sequence * array = (track_msgs__msg__LaneBoundaryArray__Sequence *)allocator.allocate(sizeof(track_msgs__msg__LaneBoundaryArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = track_msgs__msg__LaneBoundaryArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
track_msgs__msg__LaneBoundaryArray__Sequence__destroy(track_msgs__msg__LaneBoundaryArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    track_msgs__msg__LaneBoundaryArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
track_msgs__msg__LaneBoundaryArray__Sequence__are_equal(const track_msgs__msg__LaneBoundaryArray__Sequence * lhs, const track_msgs__msg__LaneBoundaryArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!track_msgs__msg__LaneBoundaryArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
track_msgs__msg__LaneBoundaryArray__Sequence__copy(
  const track_msgs__msg__LaneBoundaryArray__Sequence * input,
  track_msgs__msg__LaneBoundaryArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(track_msgs__msg__LaneBoundaryArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    track_msgs__msg__LaneBoundaryArray * data =
      (track_msgs__msg__LaneBoundaryArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!track_msgs__msg__LaneBoundaryArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          track_msgs__msg__LaneBoundaryArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!track_msgs__msg__LaneBoundaryArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
