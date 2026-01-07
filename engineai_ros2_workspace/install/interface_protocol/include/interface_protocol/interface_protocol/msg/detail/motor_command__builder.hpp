// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{

namespace builder
{

class Init_MotorCommand_damping
{
public:
  explicit Init_MotorCommand_damping(::interface_protocol::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  ::interface_protocol::msg::MotorCommand damping(::interface_protocol::msg::MotorCommand::_damping_type arg)
  {
    msg_.damping = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::msg::MotorCommand msg_;
};

class Init_MotorCommand_stiffness
{
public:
  explicit Init_MotorCommand_stiffness(::interface_protocol::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_damping stiffness(::interface_protocol::msg::MotorCommand::_stiffness_type arg)
  {
    msg_.stiffness = std::move(arg);
    return Init_MotorCommand_damping(msg_);
  }

private:
  ::interface_protocol::msg::MotorCommand msg_;
};

class Init_MotorCommand_torque
{
public:
  explicit Init_MotorCommand_torque(::interface_protocol::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_stiffness torque(::interface_protocol::msg::MotorCommand::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_MotorCommand_stiffness(msg_);
  }

private:
  ::interface_protocol::msg::MotorCommand msg_;
};

class Init_MotorCommand_feed_forward_torque
{
public:
  explicit Init_MotorCommand_feed_forward_torque(::interface_protocol::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_torque feed_forward_torque(::interface_protocol::msg::MotorCommand::_feed_forward_torque_type arg)
  {
    msg_.feed_forward_torque = std::move(arg);
    return Init_MotorCommand_torque(msg_);
  }

private:
  ::interface_protocol::msg::MotorCommand msg_;
};

class Init_MotorCommand_velocity
{
public:
  explicit Init_MotorCommand_velocity(::interface_protocol::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_feed_forward_torque velocity(::interface_protocol::msg::MotorCommand::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MotorCommand_feed_forward_torque(msg_);
  }

private:
  ::interface_protocol::msg::MotorCommand msg_;
};

class Init_MotorCommand_position
{
public:
  explicit Init_MotorCommand_position(::interface_protocol::msg::MotorCommand & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_velocity position(::interface_protocol::msg::MotorCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MotorCommand_velocity(msg_);
  }

private:
  ::interface_protocol::msg::MotorCommand msg_;
};

class Init_MotorCommand_header
{
public:
  Init_MotorCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_position header(::interface_protocol::msg::MotorCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorCommand_position(msg_);
  }

private:
  ::interface_protocol::msg::MotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::MotorCommand>()
{
  return interface_protocol::msg::builder::Init_MotorCommand_header();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
