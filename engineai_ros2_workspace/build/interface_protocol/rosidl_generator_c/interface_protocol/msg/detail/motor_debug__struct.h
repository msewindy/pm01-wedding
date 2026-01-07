// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_protocol:msg/MotorDebug.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__STRUCT_H_
#define INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'tau_cmd'
// Member 'mos_temperature'
// Member 'motor_temperature'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MotorDebug in the package interface_protocol.
typedef struct interface_protocol__msg__MotorDebug
{
  rosidl_runtime_c__double__Sequence tau_cmd;
  rosidl_runtime_c__double__Sequence mos_temperature;
  rosidl_runtime_c__double__Sequence motor_temperature;
} interface_protocol__msg__MotorDebug;

// Struct for a sequence of interface_protocol__msg__MotorDebug.
typedef struct interface_protocol__msg__MotorDebug__Sequence
{
  interface_protocol__msg__MotorDebug * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_protocol__msg__MotorDebug__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__STRUCT_H_
