// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interface_protocol:msg/GamepadKeys.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interface_protocol/msg/detail/gamepad_keys__rosidl_typesupport_introspection_c.h"
#include "interface_protocol/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interface_protocol/msg/detail/gamepad_keys__functions.h"
#include "interface_protocol/msg/detail/gamepad_keys__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface_protocol__msg__GamepadKeys__init(message_memory);
}

void interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_fini_function(void * message_memory)
{
  interface_protocol__msg__GamepadKeys__fini(message_memory);
}

size_t interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__size_function__GamepadKeys__digital_states(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_const_function__GamepadKeys__digital_states(
  const void * untyped_member, size_t index)
{
  const int32_t * member =
    (const int32_t *)(untyped_member);
  return &member[index];
}

void * interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_function__GamepadKeys__digital_states(
  void * untyped_member, size_t index)
{
  int32_t * member =
    (int32_t *)(untyped_member);
  return &member[index];
}

void interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__fetch_function__GamepadKeys__digital_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_const_function__GamepadKeys__digital_states(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__assign_function__GamepadKeys__digital_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_function__GamepadKeys__digital_states(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

size_t interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__size_function__GamepadKeys__analog_states(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_const_function__GamepadKeys__analog_states(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_function__GamepadKeys__analog_states(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__fetch_function__GamepadKeys__analog_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_const_function__GamepadKeys__analog_states(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__assign_function__GamepadKeys__analog_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_function__GamepadKeys__analog_states(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__GamepadKeys, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "digital_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__GamepadKeys, digital_states),  // bytes offset in struct
    NULL,  // default value
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__size_function__GamepadKeys__digital_states,  // size() function pointer
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_const_function__GamepadKeys__digital_states,  // get_const(index) function pointer
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_function__GamepadKeys__digital_states,  // get(index) function pointer
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__fetch_function__GamepadKeys__digital_states,  // fetch(index, &value) function pointer
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__assign_function__GamepadKeys__digital_states,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "analog_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__GamepadKeys, analog_states),  // bytes offset in struct
    NULL,  // default value
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__size_function__GamepadKeys__analog_states,  // size() function pointer
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_const_function__GamepadKeys__analog_states,  // get_const(index) function pointer
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__get_function__GamepadKeys__analog_states,  // get(index) function pointer
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__fetch_function__GamepadKeys__analog_states,  // fetch(index, &value) function pointer
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__assign_function__GamepadKeys__analog_states,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_message_members = {
  "interface_protocol__msg",  // message namespace
  "GamepadKeys",  // message name
  3,  // number of fields
  sizeof(interface_protocol__msg__GamepadKeys),
  interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_message_member_array,  // message members
  interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_init_function,  // function to initialize message memory (memory has to be allocated)
  interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_message_type_support_handle = {
  0,
  &interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface_protocol
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_protocol, msg, GamepadKeys)() {
  interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_message_type_support_handle.typesupport_identifier) {
    interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interface_protocol__msg__GamepadKeys__rosidl_typesupport_introspection_c__GamepadKeys_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
