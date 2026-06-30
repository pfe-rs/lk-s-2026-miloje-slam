// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lidar_msgs:msg/LidarSweep.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lidar_msgs/msg/detail/lidar_sweep__rosidl_typesupport_introspection_c.h"
#include "lidar_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lidar_msgs/msg/detail/lidar_sweep__functions.h"
#include "lidar_msgs/msg/detail/lidar_sweep__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `angles`
// Member `distances`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lidar_msgs__msg__LidarSweep__init(message_memory);
}

void lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_fini_function(void * message_memory)
{
  lidar_msgs__msg__LidarSweep__fini(message_memory);
}

size_t lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__size_function__LidarSweep__angles(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_const_function__LidarSweep__angles(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_function__LidarSweep__angles(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__fetch_function__LidarSweep__angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_const_function__LidarSweep__angles(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__assign_function__LidarSweep__angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_function__LidarSweep__angles(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__resize_function__LidarSweep__angles(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__size_function__LidarSweep__distances(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_const_function__LidarSweep__distances(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_function__LidarSweep__distances(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__fetch_function__LidarSweep__distances(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_const_function__LidarSweep__distances(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__assign_function__LidarSweep__distances(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_function__LidarSweep__distances(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__resize_function__LidarSweep__distances(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_msgs__msg__LidarSweep, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angles",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_msgs__msg__LidarSweep, angles),  // bytes offset in struct
    NULL,  // default value
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__size_function__LidarSweep__angles,  // size() function pointer
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_const_function__LidarSweep__angles,  // get_const(index) function pointer
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_function__LidarSweep__angles,  // get(index) function pointer
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__fetch_function__LidarSweep__angles,  // fetch(index, &value) function pointer
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__assign_function__LidarSweep__angles,  // assign(index, value) function pointer
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__resize_function__LidarSweep__angles  // resize(index) function pointer
  },
  {
    "distances",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_msgs__msg__LidarSweep, distances),  // bytes offset in struct
    NULL,  // default value
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__size_function__LidarSweep__distances,  // size() function pointer
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_const_function__LidarSweep__distances,  // get_const(index) function pointer
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__get_function__LidarSweep__distances,  // get(index) function pointer
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__fetch_function__LidarSweep__distances,  // fetch(index, &value) function pointer
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__assign_function__LidarSweep__distances,  // assign(index, value) function pointer
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__resize_function__LidarSweep__distances  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_message_members = {
  "lidar_msgs__msg",  // message namespace
  "LidarSweep",  // message name
  3,  // number of fields
  sizeof(lidar_msgs__msg__LidarSweep),
  lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_message_member_array,  // message members
  lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_init_function,  // function to initialize message memory (memory has to be allocated)
  lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_message_type_support_handle = {
  0,
  &lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidar_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_msgs, msg, LidarSweep)() {
  lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_message_type_support_handle.typesupport_identifier) {
    lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lidar_msgs__msg__LidarSweep__rosidl_typesupport_introspection_c__LidarSweep_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
