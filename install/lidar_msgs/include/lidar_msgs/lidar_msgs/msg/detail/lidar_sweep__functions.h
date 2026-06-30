// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from lidar_msgs:msg/LidarSweep.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__FUNCTIONS_H_
#define LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "lidar_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "lidar_msgs/msg/detail/lidar_sweep__struct.h"

/// Initialize msg/LidarSweep message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lidar_msgs__msg__LidarSweep
 * )) before or use
 * lidar_msgs__msg__LidarSweep__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
bool
lidar_msgs__msg__LidarSweep__init(lidar_msgs__msg__LidarSweep * msg);

/// Finalize msg/LidarSweep message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
void
lidar_msgs__msg__LidarSweep__fini(lidar_msgs__msg__LidarSweep * msg);

/// Create msg/LidarSweep message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lidar_msgs__msg__LidarSweep__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
lidar_msgs__msg__LidarSweep *
lidar_msgs__msg__LidarSweep__create();

/// Destroy msg/LidarSweep message.
/**
 * It calls
 * lidar_msgs__msg__LidarSweep__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
void
lidar_msgs__msg__LidarSweep__destroy(lidar_msgs__msg__LidarSweep * msg);

/// Check for msg/LidarSweep message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
bool
lidar_msgs__msg__LidarSweep__are_equal(const lidar_msgs__msg__LidarSweep * lhs, const lidar_msgs__msg__LidarSweep * rhs);

/// Copy a msg/LidarSweep message.
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
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
bool
lidar_msgs__msg__LidarSweep__copy(
  const lidar_msgs__msg__LidarSweep * input,
  lidar_msgs__msg__LidarSweep * output);

/// Initialize array of msg/LidarSweep messages.
/**
 * It allocates the memory for the number of elements and calls
 * lidar_msgs__msg__LidarSweep__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
bool
lidar_msgs__msg__LidarSweep__Sequence__init(lidar_msgs__msg__LidarSweep__Sequence * array, size_t size);

/// Finalize array of msg/LidarSweep messages.
/**
 * It calls
 * lidar_msgs__msg__LidarSweep__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
void
lidar_msgs__msg__LidarSweep__Sequence__fini(lidar_msgs__msg__LidarSweep__Sequence * array);

/// Create array of msg/LidarSweep messages.
/**
 * It allocates the memory for the array and calls
 * lidar_msgs__msg__LidarSweep__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
lidar_msgs__msg__LidarSweep__Sequence *
lidar_msgs__msg__LidarSweep__Sequence__create(size_t size);

/// Destroy array of msg/LidarSweep messages.
/**
 * It calls
 * lidar_msgs__msg__LidarSweep__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
void
lidar_msgs__msg__LidarSweep__Sequence__destroy(lidar_msgs__msg__LidarSweep__Sequence * array);

/// Check for msg/LidarSweep message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
bool
lidar_msgs__msg__LidarSweep__Sequence__are_equal(const lidar_msgs__msg__LidarSweep__Sequence * lhs, const lidar_msgs__msg__LidarSweep__Sequence * rhs);

/// Copy an array of msg/LidarSweep messages.
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
ROSIDL_GENERATOR_C_PUBLIC_lidar_msgs
bool
lidar_msgs__msg__LidarSweep__Sequence__copy(
  const lidar_msgs__msg__LidarSweep__Sequence * input,
  lidar_msgs__msg__LidarSweep__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__FUNCTIONS_H_
