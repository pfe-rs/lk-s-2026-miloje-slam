// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_msgs:msg/LidarSweep.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__STRUCT_H_
#define LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'angles'
// Member 'distances'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/LidarSweep in the package lidar_msgs.
typedef struct lidar_msgs__msg__LidarSweep
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence angles;
  rosidl_runtime_c__double__Sequence distances;
} lidar_msgs__msg__LidarSweep;

// Struct for a sequence of lidar_msgs__msg__LidarSweep.
typedef struct lidar_msgs__msg__LidarSweep__Sequence
{
  lidar_msgs__msg__LidarSweep * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_msgs__msg__LidarSweep__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_MSGS__MSG__DETAIL__LIDAR_SWEEP__STRUCT_H_
