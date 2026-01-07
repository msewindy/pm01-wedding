// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from interface_protocol:msg/GamepadKeys.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__FUNCTIONS_H_
#define INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "interface_protocol/msg/rosidl_generator_c__visibility_control.h"

#include "interface_protocol/msg/detail/gamepad_keys__struct.h"

/// Initialize msg/GamepadKeys message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * interface_protocol__msg__GamepadKeys
 * )) before or use
 * interface_protocol__msg__GamepadKeys__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
bool
interface_protocol__msg__GamepadKeys__init(interface_protocol__msg__GamepadKeys * msg);

/// Finalize msg/GamepadKeys message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
void
interface_protocol__msg__GamepadKeys__fini(interface_protocol__msg__GamepadKeys * msg);

/// Create msg/GamepadKeys message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * interface_protocol__msg__GamepadKeys__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
interface_protocol__msg__GamepadKeys *
interface_protocol__msg__GamepadKeys__create();

/// Destroy msg/GamepadKeys message.
/**
 * It calls
 * interface_protocol__msg__GamepadKeys__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
void
interface_protocol__msg__GamepadKeys__destroy(interface_protocol__msg__GamepadKeys * msg);

/// Check for msg/GamepadKeys message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
bool
interface_protocol__msg__GamepadKeys__are_equal(const interface_protocol__msg__GamepadKeys * lhs, const interface_protocol__msg__GamepadKeys * rhs);

/// Copy a msg/GamepadKeys message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
bool
interface_protocol__msg__GamepadKeys__copy(
  const interface_protocol__msg__GamepadKeys * input,
  interface_protocol__msg__GamepadKeys * output);

/// Initialize array of msg/GamepadKeys messages.
/**
 * It allocates the memory for the number of elements and calls
 * interface_protocol__msg__GamepadKeys__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
bool
interface_protocol__msg__GamepadKeys__Sequence__init(interface_protocol__msg__GamepadKeys__Sequence * array, size_t size);

/// Finalize array of msg/GamepadKeys messages.
/**
 * It calls
 * interface_protocol__msg__GamepadKeys__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
void
interface_protocol__msg__GamepadKeys__Sequence__fini(interface_protocol__msg__GamepadKeys__Sequence * array);

/// Create array of msg/GamepadKeys messages.
/**
 * It allocates the memory for the array and calls
 * interface_protocol__msg__GamepadKeys__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
interface_protocol__msg__GamepadKeys__Sequence *
interface_protocol__msg__GamepadKeys__Sequence__create(size_t size);

/// Destroy array of msg/GamepadKeys messages.
/**
 * It calls
 * interface_protocol__msg__GamepadKeys__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
void
interface_protocol__msg__GamepadKeys__Sequence__destroy(interface_protocol__msg__GamepadKeys__Sequence * array);

/// Check for msg/GamepadKeys message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
bool
interface_protocol__msg__GamepadKeys__Sequence__are_equal(const interface_protocol__msg__GamepadKeys__Sequence * lhs, const interface_protocol__msg__GamepadKeys__Sequence * rhs);

/// Copy an array of msg/GamepadKeys messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_interface_protocol
bool
interface_protocol__msg__GamepadKeys__Sequence__copy(
  const interface_protocol__msg__GamepadKeys__Sequence * input,
  interface_protocol__msg__GamepadKeys__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__FUNCTIONS_H_
