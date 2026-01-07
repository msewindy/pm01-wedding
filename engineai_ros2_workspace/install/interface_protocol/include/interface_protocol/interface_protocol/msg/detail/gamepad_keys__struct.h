// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_protocol:msg/GamepadKeys.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__STRUCT_H_
#define INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LB'.
enum
{
  interface_protocol__msg__GamepadKeys__LB = 0
};

/// Constant 'RB'.
enum
{
  interface_protocol__msg__GamepadKeys__RB = 1
};

/// Constant 'A'.
enum
{
  interface_protocol__msg__GamepadKeys__A = 2
};

/// Constant 'B'.
enum
{
  interface_protocol__msg__GamepadKeys__B = 3
};

/// Constant 'X'.
enum
{
  interface_protocol__msg__GamepadKeys__X = 4
};

/// Constant 'Y'.
enum
{
  interface_protocol__msg__GamepadKeys__Y = 5
};

/// Constant 'BACK'.
enum
{
  interface_protocol__msg__GamepadKeys__BACK = 6
};

/// Constant 'START'.
enum
{
  interface_protocol__msg__GamepadKeys__START = 7
};

/// Constant 'CROSS_X_UP'.
enum
{
  interface_protocol__msg__GamepadKeys__CROSS_X_UP = 8
};

/// Constant 'CROSS_X_DOWN'.
enum
{
  interface_protocol__msg__GamepadKeys__CROSS_X_DOWN = 9
};

/// Constant 'CROSS_Y_LEFT'.
enum
{
  interface_protocol__msg__GamepadKeys__CROSS_Y_LEFT = 10
};

/// Constant 'CROSS_Y_RIGHT'.
enum
{
  interface_protocol__msg__GamepadKeys__CROSS_Y_RIGHT = 11
};

/// Constant 'LT'.
enum
{
  interface_protocol__msg__GamepadKeys__LT = 0
};

/// Constant 'RT'.
enum
{
  interface_protocol__msg__GamepadKeys__RT = 1
};

/// Constant 'LEFT_STICK_X'.
enum
{
  interface_protocol__msg__GamepadKeys__LEFT_STICK_X = 2
};

/// Constant 'LEFT_STICK_Y'.
enum
{
  interface_protocol__msg__GamepadKeys__LEFT_STICK_Y = 3
};

/// Constant 'RIGHT_STICK_X'.
enum
{
  interface_protocol__msg__GamepadKeys__RIGHT_STICK_X = 4
};

/// Constant 'RIGHT_STICK_Y'.
enum
{
  interface_protocol__msg__GamepadKeys__RIGHT_STICK_Y = 5
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/GamepadKeys in the package interface_protocol.
/**
  * Timestamp field using ROS2 header
 */
typedef struct interface_protocol__msg__GamepadKeys
{
  std_msgs__msg__Header header;
  /// Digital buttons (0 = released, 1 = pressed)
  /// Fixed size array of 12 elements
  int32_t digital_states[12];
  /// Analog inputs (range: -1.0 to 1.0)
  /// Fixed size array of 6 elements
  double analog_states[6];
} interface_protocol__msg__GamepadKeys;

// Struct for a sequence of interface_protocol__msg__GamepadKeys.
typedef struct interface_protocol__msg__GamepadKeys__Sequence
{
  interface_protocol__msg__GamepadKeys * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_protocol__msg__GamepadKeys__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__STRUCT_H_
