// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/GamepadKeys.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/gamepad_keys__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{

namespace builder
{

class Init_GamepadKeys_analog_states
{
public:
  explicit Init_GamepadKeys_analog_states(::interface_protocol::msg::GamepadKeys & msg)
  : msg_(msg)
  {}
  ::interface_protocol::msg::GamepadKeys analog_states(::interface_protocol::msg::GamepadKeys::_analog_states_type arg)
  {
    msg_.analog_states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::msg::GamepadKeys msg_;
};

class Init_GamepadKeys_digital_states
{
public:
  explicit Init_GamepadKeys_digital_states(::interface_protocol::msg::GamepadKeys & msg)
  : msg_(msg)
  {}
  Init_GamepadKeys_analog_states digital_states(::interface_protocol::msg::GamepadKeys::_digital_states_type arg)
  {
    msg_.digital_states = std::move(arg);
    return Init_GamepadKeys_analog_states(msg_);
  }

private:
  ::interface_protocol::msg::GamepadKeys msg_;
};

class Init_GamepadKeys_header
{
public:
  Init_GamepadKeys_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GamepadKeys_digital_states header(::interface_protocol::msg::GamepadKeys::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GamepadKeys_digital_states(msg_);
  }

private:
  ::interface_protocol::msg::GamepadKeys msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::GamepadKeys>()
{
  return interface_protocol::msg::builder::Init_GamepadKeys_header();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__BUILDER_HPP_
