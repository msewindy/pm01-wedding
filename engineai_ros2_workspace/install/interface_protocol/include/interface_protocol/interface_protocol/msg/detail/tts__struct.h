// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_protocol:msg/Tts.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__TTS__STRUCT_H_
#define INTERFACE_PROTOCOL__MSG__DETAIL__TTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'text'
// Member 'language'
// Member 'speaker'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Tts in the package interface_protocol.
/**
  * Must, Speech content
 */
typedef struct interface_protocol__msg__Tts
{
  rosidl_runtime_c__String text;
  /// Optional, Default: en
  rosidl_runtime_c__String language;
  /// Optional, Default: default
  rosidl_runtime_c__String speaker;
  /// Optional, Default: 150
  int64_t rate;
} interface_protocol__msg__Tts;

// Struct for a sequence of interface_protocol__msg__Tts.
typedef struct interface_protocol__msg__Tts__Sequence
{
  interface_protocol__msg__Tts * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_protocol__msg__Tts__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__TTS__STRUCT_H_
