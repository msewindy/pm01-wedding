// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from interface_protocol:msg/MotorDebug.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "interface_protocol/msg/detail/motor_debug__struct.hpp"
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

void MotorDebug_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) interface_protocol::msg::MotorDebug(_init);
}

void MotorDebug_fini_function(void * message_memory)
{
  auto typed_message = static_cast<interface_protocol::msg::MotorDebug *>(message_memory);
  typed_message->~MotorDebug();
}

size_t size_function__MotorDebug__tau_cmd(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorDebug__tau_cmd(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorDebug__tau_cmd(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorDebug__tau_cmd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorDebug__tau_cmd(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorDebug__tau_cmd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorDebug__tau_cmd(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__MotorDebug__tau_cmd(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorDebug__mos_temperature(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorDebug__mos_temperature(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorDebug__mos_temperature(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorDebug__mos_temperature(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorDebug__mos_temperature(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorDebug__mos_temperature(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorDebug__mos_temperature(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__MotorDebug__mos_temperature(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotorDebug__motor_temperature(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorDebug__motor_temperature(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorDebug__motor_temperature(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorDebug__motor_temperature(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MotorDebug__motor_temperature(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MotorDebug__motor_temperature(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MotorDebug__motor_temperature(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__MotorDebug__motor_temperature(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorDebug_message_member_array[3] = {
  {
    "tau_cmd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol::msg::MotorDebug, tau_cmd),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorDebug__tau_cmd,  // size() function pointer
    get_const_function__MotorDebug__tau_cmd,  // get_const(index) function pointer
    get_function__MotorDebug__tau_cmd,  // get(index) function pointer
    fetch_function__MotorDebug__tau_cmd,  // fetch(index, &value) function pointer
    assign_function__MotorDebug__tau_cmd,  // assign(index, value) function pointer
    resize_function__MotorDebug__tau_cmd  // resize(index) function pointer
  },
  {
    "mos_temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol::msg::MotorDebug, mos_temperature),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorDebug__mos_temperature,  // size() function pointer
    get_const_function__MotorDebug__mos_temperature,  // get_const(index) function pointer
    get_function__MotorDebug__mos_temperature,  // get(index) function pointer
    fetch_function__MotorDebug__mos_temperature,  // fetch(index, &value) function pointer
    assign_function__MotorDebug__mos_temperature,  // assign(index, value) function pointer
    resize_function__MotorDebug__mos_temperature  // resize(index) function pointer
  },
  {
    "motor_temperature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol::msg::MotorDebug, motor_temperature),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorDebug__motor_temperature,  // size() function pointer
    get_const_function__MotorDebug__motor_temperature,  // get_const(index) function pointer
    get_function__MotorDebug__motor_temperature,  // get(index) function pointer
    fetch_function__MotorDebug__motor_temperature,  // fetch(index, &value) function pointer
    assign_function__MotorDebug__motor_temperature,  // assign(index, value) function pointer
    resize_function__MotorDebug__motor_temperature  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorDebug_message_members = {
  "interface_protocol::msg",  // message namespace
  "MotorDebug",  // message name
  3,  // number of fields
  sizeof(interface_protocol::msg::MotorDebug),
  MotorDebug_message_member_array,  // message members
  MotorDebug_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorDebug_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorDebug_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorDebug_message_members,
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
get_message_type_support_handle<interface_protocol::msg::MotorDebug>()
{
  return &::interface_protocol::msg::rosidl_typesupport_introspection_cpp::MotorDebug_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, interface_protocol, msg, MotorDebug)() {
  return &::interface_protocol::msg::rosidl_typesupport_introspection_cpp::MotorDebug_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
