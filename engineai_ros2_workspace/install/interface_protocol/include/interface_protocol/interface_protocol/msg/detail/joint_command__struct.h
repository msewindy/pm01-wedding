// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_protocol:msg/JointCommand.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__JOINT_COMMAND__STRUCT_H_
#define INTERFACE_PROTOCOL__MSG__DETAIL__JOINT_COMMAND__STRUCT_H_

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
// Member 'position'
// Member 'velocity'
// Member 'feed_forward_torque'
// Member 'torque'
// Member 'stiffness'
// Member 'damping'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/JointCommand in the package interface_protocol.
typedef struct interface_protocol__msg__JointCommand
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence position;
  rosidl_runtime_c__double__Sequence velocity;
  rosidl_runtime_c__double__Sequence feed_forward_torque;
  rosidl_runtime_c__double__Sequence torque;
  rosidl_runtime_c__double__Sequence stiffness;
  rosidl_runtime_c__double__Sequence damping;
  uint8_t parallel_parser_type;
} interface_protocol__msg__JointCommand;

// Struct for a sequence of interface_protocol__msg__JointCommand.
typedef struct interface_protocol__msg__JointCommand__Sequence
{
  interface_protocol__msg__JointCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_protocol__msg__JointCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__JOINT_COMMAND__STRUCT_H_
