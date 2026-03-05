// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from erp42_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice
#include "erp42_msgs/msg/detail/feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
erp42_msgs__msg__Feedback__init(erp42_msgs__msg__Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    erp42_msgs__msg__Feedback__fini(msg);
    return false;
  }
  // manual_mode
  // emergency_stop
  // gear
  // speed
  // steering
  // brake
  // encoder_count
  // heartbeat
  return true;
}

void
erp42_msgs__msg__Feedback__fini(erp42_msgs__msg__Feedback * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // manual_mode
  // emergency_stop
  // gear
  // speed
  // steering
  // brake
  // encoder_count
  // heartbeat
}

bool
erp42_msgs__msg__Feedback__are_equal(const erp42_msgs__msg__Feedback * lhs, const erp42_msgs__msg__Feedback * rhs)
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
  // manual_mode
  if (lhs->manual_mode != rhs->manual_mode) {
    return false;
  }
  // emergency_stop
  if (lhs->emergency_stop != rhs->emergency_stop) {
    return false;
  }
  // gear
  if (lhs->gear != rhs->gear) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // steering
  if (lhs->steering != rhs->steering) {
    return false;
  }
  // brake
  if (lhs->brake != rhs->brake) {
    return false;
  }
  // encoder_count
  if (lhs->encoder_count != rhs->encoder_count) {
    return false;
  }
  // heartbeat
  if (lhs->heartbeat != rhs->heartbeat) {
    return false;
  }
  return true;
}

bool
erp42_msgs__msg__Feedback__copy(
  const erp42_msgs__msg__Feedback * input,
  erp42_msgs__msg__Feedback * output)
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
  // manual_mode
  output->manual_mode = input->manual_mode;
  // emergency_stop
  output->emergency_stop = input->emergency_stop;
  // gear
  output->gear = input->gear;
  // speed
  output->speed = input->speed;
  // steering
  output->steering = input->steering;
  // brake
  output->brake = input->brake;
  // encoder_count
  output->encoder_count = input->encoder_count;
  // heartbeat
  output->heartbeat = input->heartbeat;
  return true;
}

erp42_msgs__msg__Feedback *
erp42_msgs__msg__Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  erp42_msgs__msg__Feedback * msg = (erp42_msgs__msg__Feedback *)allocator.allocate(sizeof(erp42_msgs__msg__Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(erp42_msgs__msg__Feedback));
  bool success = erp42_msgs__msg__Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
erp42_msgs__msg__Feedback__destroy(erp42_msgs__msg__Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    erp42_msgs__msg__Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
erp42_msgs__msg__Feedback__Sequence__init(erp42_msgs__msg__Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  erp42_msgs__msg__Feedback * data = NULL;

  if (size) {
    data = (erp42_msgs__msg__Feedback *)allocator.zero_allocate(size, sizeof(erp42_msgs__msg__Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = erp42_msgs__msg__Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        erp42_msgs__msg__Feedback__fini(&data[i - 1]);
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
erp42_msgs__msg__Feedback__Sequence__fini(erp42_msgs__msg__Feedback__Sequence * array)
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
      erp42_msgs__msg__Feedback__fini(&array->data[i]);
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

erp42_msgs__msg__Feedback__Sequence *
erp42_msgs__msg__Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  erp42_msgs__msg__Feedback__Sequence * array = (erp42_msgs__msg__Feedback__Sequence *)allocator.allocate(sizeof(erp42_msgs__msg__Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = erp42_msgs__msg__Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
erp42_msgs__msg__Feedback__Sequence__destroy(erp42_msgs__msg__Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    erp42_msgs__msg__Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
erp42_msgs__msg__Feedback__Sequence__are_equal(const erp42_msgs__msg__Feedback__Sequence * lhs, const erp42_msgs__msg__Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!erp42_msgs__msg__Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
erp42_msgs__msg__Feedback__Sequence__copy(
  const erp42_msgs__msg__Feedback__Sequence * input,
  erp42_msgs__msg__Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(erp42_msgs__msg__Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    erp42_msgs__msg__Feedback * data =
      (erp42_msgs__msg__Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!erp42_msgs__msg__Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          erp42_msgs__msg__Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!erp42_msgs__msg__Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
