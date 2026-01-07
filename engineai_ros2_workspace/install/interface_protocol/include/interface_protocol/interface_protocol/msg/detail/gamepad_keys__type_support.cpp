// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from interface_protocol:msg/GamepadKeys.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "interface_protocol/msg/detail/gamepad_keys__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace interface_protocol
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void GamepadKeys_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) interface_protocol::msg::GamepadKeys(_init);
}

void GamepadKeys_fini_function(void * message_memory)
{
  auto typed_message = static_cast<interface_protocol::msg::GamepadKeys *>(message_memory);
  typed_message->~GamepadKeys();
}

size_t size_function__GamepadKeys__digital_states(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__GamepadKeys__digital_states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int32_t, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__GamepadKeys__digital_states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int32_t, 12> *>(untyped_member);
  return &member[index];
}

void fetch_function__GamepadKeys__digital_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__GamepadKeys__digital_states(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__GamepadKeys__digital_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__GamepadKeys__digital_states(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

size_t size_function__GamepadKeys__analog_states(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__GamepadKeys__analog_states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__GamepadKeys__analog_states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__GamepadKeys__analog_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__GamepadKeys__analog_states(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__GamepadKeys__analog_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__GamepadKeys__analog_states(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GamepadKeys_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol::msg::GamepadKeys, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "digital_states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(interface_protocol::msg::GamepadKeys, digital_states),  // bytes offset in struct
    nullptr,  // default value
    size_function__GamepadKeys__digital_states,  // size() function pointer
    get_const_function__GamepadKeys__digital_states,  // get_const(index) function pointer
    get_function__GamepadKeys__digital_states,  // get(index) function pointer
    fetch_function__GamepadKeys__digital_states,  // fetch(index, &value) function pointer
    assign_function__GamepadKeys__digital_states,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "analog_states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(interface_protocol::msg::GamepadKeys, analog_states),  // bytes offset in struct
    nullptr,  // default value
    size_function__GamepadKeys__analog_states,  // size() function pointer
    get_const_function__GamepadKeys__analog_states,  // get_const(index) function pointer
    get_function__GamepadKeys__analog_states,  // get(index) function pointer
    fetch_function__GamepadKeys__analog_states,  // fetch(index, &value) function pointer
    assign_function__GamepadKeys__analog_states,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GamepadKeys_message_members = {
  "interface_protocol::msg",  // message namespace
  "GamepadKeys",  // message name
  3,  // number of fields
  sizeof(interface_protocol::msg::GamepadKeys),
  GamepadKeys_message_member_array,  // message members
  GamepadKeys_init_function,  // function to initialize message memory (memory has to be allocated)
  GamepadKeys_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GamepadKeys_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GamepadKeys_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace interface_protocol


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<interface_protocol::msg::GamepadKeys>()
{
  return &::interface_protocol::msg::rosidl_typesupport_introspection_cpp::GamepadKeys_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, interface_protocol, msg, GamepadKeys)() {
  return &::interface_protocol::msg::rosidl_typesupport_introspection_cpp::GamepadKeys_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
