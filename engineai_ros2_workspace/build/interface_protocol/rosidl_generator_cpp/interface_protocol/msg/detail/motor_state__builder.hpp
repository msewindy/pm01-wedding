// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/MotorState.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{

namespace builder
{

class Init_MotorState_torque
{
public:
  explicit Init_MotorState_torque(::interface_protocol::msg::MotorState & msg)
  : msg_(msg)
  {}
  ::interface_protocol::msg::MotorState torque(::interface_protocol::msg::MotorState::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::msg::MotorState msg_;
};

class Init_MotorState_velocity
{
public:
  explicit Init_MotorState_velocity(::interface_protocol::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_torque velocity(::interface_protocol::msg::MotorState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MotorState_torque(msg_);
  }

private:
  ::interface_protocol::msg::MotorState msg_;
};

class Init_MotorState_position
{
public:
  explicit Init_MotorState_position(::interface_protocol::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_velocity position(::interface_protocol::msg::MotorState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MotorState_velocity(msg_);
  }

private:
  ::interface_protocol::msg::MotorState msg_;
};

class Init_MotorState_header
{
public:
  Init_MotorState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorState_position header(::interface_protocol::msg::MotorState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorState_position(msg_);
  }

private:
  ::interface_protocol::msg::MotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::MotorState>()
{
  return interface_protocol::msg::builder::Init_MotorState_header();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
