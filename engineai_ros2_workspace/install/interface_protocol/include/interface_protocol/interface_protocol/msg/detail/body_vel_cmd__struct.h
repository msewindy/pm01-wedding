// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_protocol:msg/BodyVelCmd.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__BODY_VEL_CMD__STRUCT_H_
#define INTERFACE_PROTOCOL__MSG__DETAIL__BODY_VEL_CMD__STRUCT_H_

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
// Member 'linear_velocity'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/BodyVelCmd in the package interface_protocol.
typedef struct interface_protocol__msg__BodyVelCmd
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence linear_velocity;
  double yaw_velocity;
} interface_protocol__msg__BodyVelCmd;

// Struct for a sequence of interface_protocol__msg__BodyVelCmd.
typedef struct interface_protocol__msg__BodyVelCmd__Sequence
{
  interface_protocol__msg__BodyVelCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_protocol__msg__BodyVelCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__BODY_VEL_CMD__STRUCT_H_
