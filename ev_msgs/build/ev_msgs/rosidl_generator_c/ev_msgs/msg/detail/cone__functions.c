// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ev_msgs:msg/Cone.idl
// generated code does not contain a copyright notice
#include "ev_msgs/msg/detail/cone__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
ev_msgs__msg__Cone__init(ev_msgs__msg__Cone * msg)
{
  if (!msg) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    ev_msgs__msg__Cone__fini(msg);
    return false;
  }
  // radius
  // height
  // confidence
  // label
  return true;
}

void
ev_msgs__msg__Cone__fini(ev_msgs__msg__Cone * msg)
{
  if (!msg) {
    return;
  }
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // radius
  // height
  // confidence
  // label
}

bool
ev_msgs__msg__Cone__are_equal(const ev_msgs__msg__Cone * lhs, const ev_msgs__msg__Cone * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // radius
  if (lhs->radius != rhs->radius) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // label
  if (lhs->label != rhs->label) {
    return false;
  }
  return true;
}

bool
ev_msgs__msg__Cone__copy(
  const ev_msgs__msg__Cone * input,
  ev_msgs__msg__Cone * output)
{
  if (!input || !output) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // radius
  output->radius = input->radius;
  // height
  output->height = input->height;
  // confidence
  output->confidence = input->confidence;
  // label
  output->label = input->label;
  return true;
}

ev_msgs__msg__Cone *
ev_msgs__msg__Cone__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ev_msgs__msg__Cone * msg = (ev_msgs__msg__Cone *)allocator.allocate(sizeof(ev_msgs__msg__Cone), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ev_msgs__msg__Cone));
  bool success = ev_msgs__msg__Cone__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ev_msgs__msg__Cone__destroy(ev_msgs__msg__Cone * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ev_msgs__msg__Cone__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ev_msgs__msg__Cone__Sequence__init(ev_msgs__msg__Cone__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ev_msgs__msg__Cone * data = NULL;

  if (size) {
    data = (ev_msgs__msg__Cone *)allocator.zero_allocate(size, sizeof(ev_msgs__msg__Cone), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ev_msgs__msg__Cone__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ev_msgs__msg__Cone__fini(&data[i - 1]);
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
ev_msgs__msg__Cone__Sequence__fini(ev_msgs__msg__Cone__Sequence * array)
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
      ev_msgs__msg__Cone__fini(&array->data[i]);
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

ev_msgs__msg__Cone__Sequence *
ev_msgs__msg__Cone__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ev_msgs__msg__Cone__Sequence * array = (ev_msgs__msg__Cone__Sequence *)allocator.allocate(sizeof(ev_msgs__msg__Cone__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ev_msgs__msg__Cone__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ev_msgs__msg__Cone__Sequence__destroy(ev_msgs__msg__Cone__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ev_msgs__msg__Cone__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ev_msgs__msg__Cone__Sequence__are_equal(const ev_msgs__msg__Cone__Sequence * lhs, const ev_msgs__msg__Cone__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ev_msgs__msg__Cone__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ev_msgs__msg__Cone__Sequence__copy(
  const ev_msgs__msg__Cone__Sequence * input,
  ev_msgs__msg__Cone__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ev_msgs__msg__Cone);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ev_msgs__msg__Cone * data =
      (ev_msgs__msg__Cone *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ev_msgs__msg__Cone__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ev_msgs__msg__Cone__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ev_msgs__msg__Cone__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
