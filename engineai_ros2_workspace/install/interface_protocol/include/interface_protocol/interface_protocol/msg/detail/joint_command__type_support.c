// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interface_protocol:msg/JointCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interface_protocol/msg/detail/joint_command__rosidl_typesupport_introspection_c.h"
#include "interface_protocol/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interface_protocol/msg/detail/joint_command__functions.h"
#include "interface_protocol/msg/detail/joint_command__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `position`
// Member `velocity`
// Member `feed_forward_torque`
// Member `torque`
// Member `stiffness`
// Member `damping`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interface_protocol__msg__JointCommand__init(message_memory);
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_fini_function(void * message_memory)
{
  interface_protocol__msg__JointCommand__fini(message_memory);
}

size_t interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__feed_forward_torque(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__feed_forward_torque(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__feed_forward_torque(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__feed_forward_torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__feed_forward_torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__feed_forward_torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__feed_forward_torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__feed_forward_torque(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__torque(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__torque(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__torque(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__torque(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__torque(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__torque(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__torque(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__torque(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__stiffness(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__stiffness(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__stiffness(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__stiffness(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__stiffness(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__stiffness(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__stiffness(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__stiffness(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__damping(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__damping(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__damping(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__damping(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__damping(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__damping(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__damping(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__damping(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__JointCommand, header),  // bytes offset in struct
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
    offsetof(interface_protocol__msg__JointCommand, position),  // bytes offset in struct
    NULL,  // default value
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__position,  // size() function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__position,  // get_const(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__position,  // get(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__position,  // fetch(index, &value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__position,  // assign(index, value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__position  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__JointCommand, velocity),  // bytes offset in struct
    NULL,  // default value
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__velocity,  // size() function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__velocity,  // get_const(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__velocity,  // get(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__velocity,  // fetch(index, &value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__velocity,  // assign(index, value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__velocity  // resize(index) function pointer
  },
  {
    "feed_forward_torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__JointCommand, feed_forward_torque),  // bytes offset in struct
    NULL,  // default value
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__feed_forward_torque,  // size() function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__feed_forward_torque,  // get_const(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__feed_forward_torque,  // get(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__feed_forward_torque,  // fetch(index, &value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__feed_forward_torque,  // assign(index, value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__feed_forward_torque  // resize(index) function pointer
  },
  {
    "torque",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__JointCommand, torque),  // bytes offset in struct
    NULL,  // default value
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__torque,  // size() function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__torque,  // get_const(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__torque,  // get(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__torque,  // fetch(index, &value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__torque,  // assign(index, value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__torque  // resize(index) function pointer
  },
  {
    "stiffness",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__JointCommand, stiffness),  // bytes offset in struct
    NULL,  // default value
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__stiffness,  // size() function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__stiffness,  // get_const(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__stiffness,  // get(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__stiffness,  // fetch(index, &value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__stiffness,  // assign(index, value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__stiffness  // resize(index) function pointer
  },
  {
    "damping",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__JointCommand, damping),  // bytes offset in struct
    NULL,  // default value
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__damping,  // size() function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__damping,  // get_const(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__damping,  // get(index) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__damping,  // fetch(index, &value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__damping,  // assign(index, value) function pointer
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__damping  // resize(index) function pointer
  },
  {
    "parallel_parser_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol__msg__JointCommand, parallel_parser_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_members = {
  "interface_protocol__msg",  // message namespace
  "JointCommand",  // message name
  8,  // number of fields
  sizeof(interface_protocol__msg__JointCommand),
  interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_member_array,  // message members
  interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle = {
  0,
  &interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interface_protocol
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_protocol, msg, JointCommand)() {
  interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle.typesupport_identifier) {
    interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interface_protocol__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
