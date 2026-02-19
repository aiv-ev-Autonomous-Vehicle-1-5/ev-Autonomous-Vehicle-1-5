// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidar_interfaces:msg/Cone.idl
// generated code does not contain a copyright notice
#include "lidar_interfaces/msg/detail/cone__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `dimensions`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
lidar_interfaces__msg__Cone__init(lidar_interfaces__msg__Cone * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    lidar_interfaces__msg__Cone__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    lidar_interfaces__msg__Cone__fini(msg);
    return false;
  }
  // dimensions
  if (!geometry_msgs__msg__Vector3__init(&msg->dimensions)) {
    lidar_interfaces__msg__Cone__fini(msg);
    return false;
  }
  // confidence
  // label
  return true;
}

void
lidar_interfaces__msg__Cone__fini(lidar_interfaces__msg__Cone * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // dimensions
  geometry_msgs__msg__Vector3__fini(&msg->dimensions);
  // confidence
  // label
}

bool
lidar_interfaces__msg__Cone__are_equal(const lidar_interfaces__msg__Cone * lhs, const lidar_interfaces__msg__Cone * rhs)
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
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // dimensions
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->dimensions), &(rhs->dimensions)))
  {
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
lidar_interfaces__msg__Cone__copy(
  const lidar_interfaces__msg__Cone * input,
  lidar_interfaces__msg__Cone * output)
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
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // dimensions
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->dimensions), &(output->dimensions)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // label
  output->label = input->label;
  return true;
}

lidar_interfaces__msg__Cone *
lidar_interfaces__msg__Cone__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_interfaces__msg__Cone * msg = (lidar_interfaces__msg__Cone *)allocator.allocate(sizeof(lidar_interfaces__msg__Cone), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_interfaces__msg__Cone));
  bool success = lidar_interfaces__msg__Cone__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidar_interfaces__msg__Cone__destroy(lidar_interfaces__msg__Cone * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidar_interfaces__msg__Cone__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidar_interfaces__msg__Cone__Sequence__init(lidar_interfaces__msg__Cone__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_interfaces__msg__Cone * data = NULL;

  if (size) {
    data = (lidar_interfaces__msg__Cone *)allocator.zero_allocate(size, sizeof(lidar_interfaces__msg__Cone), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_interfaces__msg__Cone__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_interfaces__msg__Cone__fini(&data[i - 1]);
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
lidar_interfaces__msg__Cone__Sequence__fini(lidar_interfaces__msg__Cone__Sequence * array)
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
      lidar_interfaces__msg__Cone__fini(&array->data[i]);
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

lidar_interfaces__msg__Cone__Sequence *
lidar_interfaces__msg__Cone__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_interfaces__msg__Cone__Sequence * array = (lidar_interfaces__msg__Cone__Sequence *)allocator.allocate(sizeof(lidar_interfaces__msg__Cone__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidar_interfaces__msg__Cone__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidar_interfaces__msg__Cone__Sequence__destroy(lidar_interfaces__msg__Cone__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidar_interfaces__msg__Cone__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidar_interfaces__msg__Cone__Sequence__are_equal(const lidar_interfaces__msg__Cone__Sequence * lhs, const lidar_interfaces__msg__Cone__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidar_interfaces__msg__Cone__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidar_interfaces__msg__Cone__Sequence__copy(
  const lidar_interfaces__msg__Cone__Sequence * input,
  lidar_interfaces__msg__Cone__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidar_interfaces__msg__Cone);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lidar_interfaces__msg__Cone * data =
      (lidar_interfaces__msg__Cone *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidar_interfaces__msg__Cone__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lidar_interfaces__msg__Cone__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lidar_interfaces__msg__Cone__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
