// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_protocol:msg/Heartbeat.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__HEARTBEAT__STRUCT_H_
#define INTERFACE_PROTOCOL__MSG__DETAIL__HEARTBEAT__STRUCT_H_

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
// Member 'node_name'
// Member 'node_status'
// Member 'error_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Heartbeat in the package interface_protocol.
typedef struct interface_protocol__msg__Heartbeat
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String node_name;
  int64_t startup_timestamp;
  /// idle, running, error, sleep
  rosidl_runtime_c__String node_status;
  int64_t error_code;
  rosidl_runtime_c__String error_message;
} interface_protocol__msg__Heartbeat;

// Struct for a sequence of interface_protocol__msg__Heartbeat.
typedef struct interface_protocol__msg__Heartbeat__Sequence
{
  interface_protocol__msg__Heartbeat * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_protocol__msg__Heartbeat__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__HEARTBEAT__STRUCT_H_
