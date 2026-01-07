// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_protocol:msg/ImuInfo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__STRUCT_H_
#define INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__STRUCT_H_

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
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'rpy'
// Member 'linear_acceleration'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/ImuInfo in the package interface_protocol.
typedef struct interface_protocol__msg__ImuInfo
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Quaternion quaternion;
  geometry_msgs__msg__Vector3 rpy;
  geometry_msgs__msg__Vector3 linear_acceleration;
  geometry_msgs__msg__Vector3 angular_velocity;
} interface_protocol__msg__ImuInfo;

// Struct for a sequence of interface_protocol__msg__ImuInfo.
typedef struct interface_protocol__msg__ImuInfo__Sequence
{
  interface_protocol__msg__ImuInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_protocol__msg__ImuInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__STRUCT_H_
