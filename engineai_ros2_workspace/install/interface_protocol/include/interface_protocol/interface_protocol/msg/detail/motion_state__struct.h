// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_protocol:msg/MotionState.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__MOTION_STATE__STRUCT_H_
#define INTERFACE_PROTOCOL__MSG__DETAIL__MOTION_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'current_motion_task'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MotionState in the package interface_protocol.
typedef struct interface_protocol__msg__MotionState
{
  rosidl_runtime_c__String current_motion_task;
} interface_protocol__msg__MotionState;

// Struct for a sequence of interface_protocol__msg__MotionState.
typedef struct interface_protocol__msg__MotionState__Sequence
{
  interface_protocol__msg__MotionState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_protocol__msg__MotionState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__MOTION_STATE__STRUCT_H_
