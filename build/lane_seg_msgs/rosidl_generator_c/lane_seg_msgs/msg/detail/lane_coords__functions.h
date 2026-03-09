// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from lane_seg_msgs:msg/LaneCoords.idl
// generated code does not contain a copyright notice

#ifndef LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__FUNCTIONS_H_
#define LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "lane_seg_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "lane_seg_msgs/msg/detail/lane_coords__struct.h"

/// Initialize msg/LaneCoords message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lane_seg_msgs__msg__LaneCoords
 * )) before or use
 * lane_seg_msgs__msg__LaneCoords__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
bool
lane_seg_msgs__msg__LaneCoords__init(lane_seg_msgs__msg__LaneCoords * msg);

/// Finalize msg/LaneCoords message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
void
lane_seg_msgs__msg__LaneCoords__fini(lane_seg_msgs__msg__LaneCoords * msg);

/// Create msg/LaneCoords message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lane_seg_msgs__msg__LaneCoords__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
lane_seg_msgs__msg__LaneCoords *
lane_seg_msgs__msg__LaneCoords__create();

/// Destroy msg/LaneCoords message.
/**
 * It calls
 * lane_seg_msgs__msg__LaneCoords__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
void
lane_seg_msgs__msg__LaneCoords__destroy(lane_seg_msgs__msg__LaneCoords * msg);

/// Check for msg/LaneCoords message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
bool
lane_seg_msgs__msg__LaneCoords__are_equal(const lane_seg_msgs__msg__LaneCoords * lhs, const lane_seg_msgs__msg__LaneCoords * rhs);

/// Copy a msg/LaneCoords message.
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
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
bool
lane_seg_msgs__msg__LaneCoords__copy(
  const lane_seg_msgs__msg__LaneCoords * input,
  lane_seg_msgs__msg__LaneCoords * output);

/// Initialize array of msg/LaneCoords messages.
/**
 * It allocates the memory for the number of elements and calls
 * lane_seg_msgs__msg__LaneCoords__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
bool
lane_seg_msgs__msg__LaneCoords__Sequence__init(lane_seg_msgs__msg__LaneCoords__Sequence * array, size_t size);

/// Finalize array of msg/LaneCoords messages.
/**
 * It calls
 * lane_seg_msgs__msg__LaneCoords__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
void
lane_seg_msgs__msg__LaneCoords__Sequence__fini(lane_seg_msgs__msg__LaneCoords__Sequence * array);

/// Create array of msg/LaneCoords messages.
/**
 * It allocates the memory for the array and calls
 * lane_seg_msgs__msg__LaneCoords__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
lane_seg_msgs__msg__LaneCoords__Sequence *
lane_seg_msgs__msg__LaneCoords__Sequence__create(size_t size);

/// Destroy array of msg/LaneCoords messages.
/**
 * It calls
 * lane_seg_msgs__msg__LaneCoords__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
void
lane_seg_msgs__msg__LaneCoords__Sequence__destroy(lane_seg_msgs__msg__LaneCoords__Sequence * array);

/// Check for msg/LaneCoords message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
bool
lane_seg_msgs__msg__LaneCoords__Sequence__are_equal(const lane_seg_msgs__msg__LaneCoords__Sequence * lhs, const lane_seg_msgs__msg__LaneCoords__Sequence * rhs);

/// Copy an array of msg/LaneCoords messages.
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
ROSIDL_GENERATOR_C_PUBLIC_lane_seg_msgs
bool
lane_seg_msgs__msg__LaneCoords__Sequence__copy(
  const lane_seg_msgs__msg__LaneCoords__Sequence * input,
  lane_seg_msgs__msg__LaneCoords__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__FUNCTIONS_H_
