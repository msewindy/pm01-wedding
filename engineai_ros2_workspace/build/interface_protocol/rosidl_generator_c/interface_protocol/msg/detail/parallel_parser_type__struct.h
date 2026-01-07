// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface_protocol:msg/ParallelParserType.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__PARALLEL_PARSER_TYPE__STRUCT_H_
#define INTERFACE_PROTOCOL__MSG__DETAIL__PARALLEL_PARSER_TYPE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CLASSIC_PARSER'.
enum
{
  interface_protocol__msg__ParallelParserType__CLASSIC_PARSER = 0
};

/// Constant 'RL_PARSER'.
enum
{
  interface_protocol__msg__ParallelParserType__RL_PARSER = 1
};

/// Struct defined in msg/ParallelParserType in the package interface_protocol.
typedef struct interface_protocol__msg__ParallelParserType
{
  uint8_t structure_needs_at_least_one_member;
} interface_protocol__msg__ParallelParserType;

// Struct for a sequence of interface_protocol__msg__ParallelParserType.
typedef struct interface_protocol__msg__ParallelParserType__Sequence
{
  interface_protocol__msg__ParallelParserType * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface_protocol__msg__ParallelParserType__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__PARALLEL_PARSER_TYPE__STRUCT_H_
