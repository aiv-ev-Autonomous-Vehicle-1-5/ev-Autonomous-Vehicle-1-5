// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ev_msgs:msg/ConeArray.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__CONE_ARRAY__FUNCTIONS_H_
#define EV_MSGS__MSG__DETAIL__CONE_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ev_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ev_msgs/msg/detail/cone_array__struct.h"

/// Initialize msg/ConeArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ev_msgs__msg__ConeArray
 * )) before or use
 * ev_msgs__msg__ConeArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
bool
ev_msgs__msg__ConeArray__init(ev_msgs__msg__ConeArray * msg);

/// Finalize msg/ConeArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
void
ev_msgs__msg__ConeArray__fini(ev_msgs__msg__ConeArray * msg);

/// Create msg/ConeArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ev_msgs__msg__ConeArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
ev_msgs__msg__ConeArray *
ev_msgs__msg__ConeArray__create();

/// Destroy msg/ConeArray message.
/**
 * It calls
 * ev_msgs__msg__ConeArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
void
ev_msgs__msg__ConeArray__destroy(ev_msgs__msg__ConeArray * msg);

/// Check for msg/ConeArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
bool
ev_msgs__msg__ConeArray__are_equal(const ev_msgs__msg__ConeArray * lhs, const ev_msgs__msg__ConeArray * rhs);

/// Copy a msg/ConeArray message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
bool
ev_msgs__msg__ConeArray__copy(
  const ev_msgs__msg__ConeArray * input,
  ev_msgs__msg__ConeArray * output);

/// Initialize array of msg/ConeArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * ev_msgs__msg__ConeArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
bool
ev_msgs__msg__ConeArray__Sequence__init(ev_msgs__msg__ConeArray__Sequence * array, size_t size);

/// Finalize array of msg/ConeArray messages.
/**
 * It calls
 * ev_msgs__msg__ConeArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
void
ev_msgs__msg__ConeArray__Sequence__fini(ev_msgs__msg__ConeArray__Sequence * array);

/// Create array of msg/ConeArray messages.
/**
 * It allocates the memory for the array and calls
 * ev_msgs__msg__ConeArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
ev_msgs__msg__ConeArray__Sequence *
ev_msgs__msg__ConeArray__Sequence__create(size_t size);

/// Destroy array of msg/ConeArray messages.
/**
 * It calls
 * ev_msgs__msg__ConeArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
void
ev_msgs__msg__ConeArray__Sequence__destroy(ev_msgs__msg__ConeArray__Sequence * array);

/// Check for msg/ConeArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
bool
ev_msgs__msg__ConeArray__Sequence__are_equal(const ev_msgs__msg__ConeArray__Sequence * lhs, const ev_msgs__msg__ConeArray__Sequence * rhs);

/// Copy an array of msg/ConeArray messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev_msgs
bool
ev_msgs__msg__ConeArray__Sequence__copy(
  const ev_msgs__msg__ConeArray__Sequence * input,
  ev_msgs__msg__ConeArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EV_MSGS__MSG__DETAIL__CONE_ARRAY__FUNCTIONS_H_
