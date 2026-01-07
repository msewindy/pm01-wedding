// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_protocol:srv/EnableMotor.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__STRUCT_H_
#define INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/EnableMotor in the package interface_protocol.
typedef struct interface_protocol__srv__EnableMotor_Request
{
  bool enable;
} interface_protocol__srv__EnableMotor_Request;

// Struct for a sequence of interface_protocol__srv__EnableMotor_Request.
typedef struct interface_protocol__srv__EnableMotor_Request__Sequence
{
  interface_protocol__srv__EnableMotor_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_protocol__srv__EnableMotor_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/EnableMotor in the package interface_protocol.
typedef struct interface_protocol__srv__EnableMotor_Response
{
  bool success;
  rosidl_runtime_c__String message;
} interface_protocol__srv__EnableMotor_Response;

// Struct for a sequence of interface_protocol__srv__EnableMotor_Response.
typedef struct interface_protocol__srv__EnableMotor_Response__Sequence
{
  interface_protocol__srv__EnableMotor_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_protocol__srv__EnableMotor_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__STRUCT_H_
