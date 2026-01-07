// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interface_protocol:msg/MotorState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interface_protocol/msg/detail/motor_state__rosidl_typesupport_introspection_c.h"
#include "interface_protocol/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interface_protocol/msg/detail/motor_state__functions.h"
#include "interface_protocol/msg/detail/motor_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `position`
// Member `velocity`
// Member `torque`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface_protocol__msg__MotorState__init(message_memory);
}

void interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_fini_function(void * message_memory)
{
  interface_protocol__msg__MotorState__fini(message_memory);
}

size_t interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__size_function__MotorState__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_const_function__MotorState__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_function__MotorState__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__fetch_function__MotorState__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_const_function__MotorState__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__assign_function__MotorState__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_function__MotorState__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__resize_function__MotorState__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__size_function__MotorState__velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_const_function__MotorState__velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_function__MotorState__velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__fetch_function__MotorState__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_const_function__MotorState__velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__assign_function__MotorState__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_function__MotorState__velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__resize_function__MotorState__velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__size_function__MotorState__torque(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_const_function__MotorState__torque(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_function__MotorState__torque(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__fetch_function__MotorState__torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_const_function__MotorState__torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__assign_function__MotorState__torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_function__MotorState__torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__resize_function__MotorState__torque(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__MotorState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__MotorState, position),  // bytes offset in struct
    NULL,  // default value
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__size_function__MotorState__position,  // size() function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_const_function__MotorState__position,  // get_const(index) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_function__MotorState__position,  // get(index) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__fetch_function__MotorState__position,  // fetch(index, &value) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__assign_function__MotorState__position,  // assign(index, value) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__resize_function__MotorState__position  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__MotorState, velocity),  // bytes offset in struct
    NULL,  // default value
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__size_function__MotorState__velocity,  // size() function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_const_function__MotorState__velocity,  // get_const(index) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_function__MotorState__velocity,  // get(index) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__fetch_function__MotorState__velocity,  // fetch(index, &value) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__assign_function__MotorState__velocity,  // assign(index, value) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__resize_function__MotorState__velocity  // resize(index) function pointer
  },
  {
    "torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__MotorState, torque),  // bytes offset in struct
    NULL,  // default value
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__size_function__MotorState__torque,  // size() function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_const_function__MotorState__torque,  // get_const(index) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__get_function__MotorState__torque,  // get(index) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__fetch_function__MotorState__torque,  // fetch(index, &value) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__assign_function__MotorState__torque,  // assign(index, value) function pointer
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__resize_function__MotorState__torque  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_members = {
  "interface_protocol__msg",  // message namespace
  "MotorState",  // message name
  4,  // number of fields
  sizeof(interface_protocol__msg__MotorState),
  interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_member_array,  // message members
  interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_init_function,  // function to initialize message memory (memory has to be allocated)
  interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_type_support_handle = {
  0,
  &interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface_protocol
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_protocol, msg, MotorState)() {
  interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_type_support_handle.typesupport_identifier) {
    interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interface_protocol__msg__MotorState__rosidl_typesupport_introspection_c__MotorState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
