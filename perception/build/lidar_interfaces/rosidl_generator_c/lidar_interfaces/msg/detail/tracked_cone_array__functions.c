// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidar_interfaces:msg/TrackedConeArray.idl
// generated code does not contain a copyright notice
#include "lidar_interfaces/msg/detail/tracked_cone_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `cones`
#include "lidar_interfaces/msg/detail/tracked_cone__functions.h"

bool
lidar_interfaces__msg__TrackedConeArray__init(lidar_interfaces__msg__TrackedConeArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    lidar_interfaces__msg__TrackedConeArray__fini(msg);
    return false;
  }
  // cones
  if (!lidar_interfaces__msg__TrackedCone__Sequence__init(&msg->cones, 0)) {
    lidar_interfaces__msg__TrackedConeArray__fini(msg);
    return false;
  }
  return true;
}

void
lidar_interfaces__msg__TrackedConeArray__fini(lidar_interfaces__msg__TrackedConeArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // cones
  lidar_interfaces__msg__TrackedCone__Sequence__fini(&msg->cones);
}

bool
lidar_interfaces__msg__TrackedConeArray__are_equal(const lidar_interfaces__msg__TrackedConeArray * lhs, const lidar_interfaces__msg__TrackedConeArray * rhs)
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
  // cones
  if (!lidar_interfaces__msg__TrackedCone__Sequence__are_equal(
      &(lhs->cones), &(rhs->cones)))
  {
    return false;
  }
  return true;
}

bool
lidar_interfaces__msg__TrackedConeArray__copy(
  const lidar_interfaces__msg__TrackedConeArray * input,
  lidar_interfaces__msg__TrackedConeArray * output)
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
  // cones
  if (!lidar_interfaces__msg__TrackedCone__Sequence__copy(
      &(input->cones), &(output->cones)))
  {
    return false;
  }
  return true;
}

lidar_interfaces__msg__TrackedConeArray *
lidar_interfaces__msg__TrackedConeArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_interfaces__msg__TrackedConeArray * msg = (lidar_interfaces__msg__TrackedConeArray *)allocator.allocate(sizeof(lidar_interfaces__msg__TrackedConeArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_interfaces__msg__TrackedConeArray));
  bool success = lidar_interfaces__msg__TrackedConeArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidar_interfaces__msg__TrackedConeArray__destroy(lidar_interfaces__msg__TrackedConeArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidar_interfaces__msg__TrackedConeArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidar_interfaces__msg__TrackedConeArray__Sequence__init(lidar_interfaces__msg__TrackedConeArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_interfaces__msg__TrackedConeArray * data = NULL;

  if (size) {
    data = (lidar_interfaces__msg__TrackedConeArray *)allocator.zero_allocate(size, sizeof(lidar_interfaces__msg__TrackedConeArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_interfaces__msg__TrackedConeArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_interfaces__msg__TrackedConeArray__fini(&data[i - 1]);
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
lidar_interfaces__msg__TrackedConeArray__Sequence__fini(lidar_interfaces__msg__TrackedConeArray__Sequence * array)
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
      lidar_interfaces__msg__TrackedConeArray__fini(&array->data[i]);
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

lidar_interfaces__msg__TrackedConeArray__Sequence *
lidar_interfaces__msg__TrackedConeArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_interfaces__msg__TrackedConeArray__Sequence * array = (lidar_interfaces__msg__TrackedConeArray__Sequence *)allocator.allocate(sizeof(lidar_interfaces__msg__TrackedConeArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidar_interfaces__msg__TrackedConeArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidar_interfaces__msg__TrackedConeArray__Sequence__destroy(lidar_interfaces__msg__TrackedConeArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidar_interfaces__msg__TrackedConeArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidar_interfaces__msg__TrackedConeArray__Sequence__are_equal(const lidar_interfaces__msg__TrackedConeArray__Sequence * lhs, const lidar_interfaces__msg__TrackedConeArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidar_interfaces__msg__TrackedConeArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidar_interfaces__msg__TrackedConeArray__Sequence__copy(
  const lidar_interfaces__msg__TrackedConeArray__Sequence * input,
  lidar_interfaces__msg__TrackedConeArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidar_interfaces__msg__TrackedConeArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lidar_interfaces__msg__TrackedConeArray * data =
      (lidar_interfaces__msg__TrackedConeArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidar_interfaces__msg__TrackedConeArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lidar_interfaces__msg__TrackedConeArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lidar_interfaces__msg__TrackedConeArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
