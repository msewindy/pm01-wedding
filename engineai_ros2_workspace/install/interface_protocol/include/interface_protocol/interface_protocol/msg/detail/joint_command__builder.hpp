// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/JointCommand.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/joint_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{

namespace builder
{

class Init_JointCommand_parallel_parser_type
{
public:
  explicit Init_JointCommand_parallel_parser_type(::interface_protocol::msg::JointCommand & msg)
  : msg_(msg)
  {}
  ::interface_protocol::msg::JointCommand parallel_parser_type(::interface_protocol::msg::JointCommand::_parallel_parser_type_type arg)
  {
    msg_.parallel_parser_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::msg::JointCommand msg_;
};

class Init_JointCommand_damping
{
public:
  explicit Init_JointCommand_damping(::interface_protocol::msg::JointCommand & msg)
  : msg_(msg)
  {}
  Init_JointCommand_parallel_parser_type damping(::interface_protocol::msg::JointCommand::_damping_type arg)
  {
    msg_.damping = std::move(arg);
    return Init_JointCommand_parallel_parser_type(msg_);
  }

private:
  ::interface_protocol::msg::JointCommand msg_;
};

class Init_JointCommand_stiffness
{
public:
  explicit Init_JointCommand_stiffness(::interface_protocol::msg::JointCommand & msg)
  : msg_(msg)
  {}
  Init_JointCommand_damping stiffness(::interface_protocol::msg::JointCommand::_stiffness_type arg)
  {
    msg_.stiffness = std::move(arg);
    return Init_JointCommand_damping(msg_);
  }

private:
  ::interface_protocol::msg::JointCommand msg_;
};

class Init_JointCommand_torque
{
public:
  explicit Init_JointCommand_torque(::interface_protocol::msg::JointCommand & msg)
  : msg_(msg)
  {}
  Init_JointCommand_stiffness torque(::interface_protocol::msg::JointCommand::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_JointCommand_stiffness(msg_);
  }

private:
  ::interface_protocol::msg::JointCommand msg_;
};

class Init_JointCommand_feed_forward_torque
{
public:
  explicit Init_JointCommand_feed_forward_torque(::interface_protocol::msg::JointCommand & msg)
  : msg_(msg)
  {}
  Init_JointCommand_torque feed_forward_torque(::interface_protocol::msg::JointCommand::_feed_forward_torque_type arg)
  {
    msg_.feed_forward_torque = std::move(arg);
    return Init_JointCommand_torque(msg_);
  }

private:
  ::interface_protocol::msg::JointCommand msg_;
};

class Init_JointCommand_velocity
{
public:
  explicit Init_JointCommand_velocity(::interface_protocol::msg::JointCommand & msg)
  : msg_(msg)
  {}
  Init_JointCommand_feed_forward_torque velocity(::interface_protocol::msg::JointCommand::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_JointCommand_feed_forward_torque(msg_);
  }

private:
  ::interface_protocol::msg::JointCommand msg_;
};

class Init_JointCommand_position
{
public:
  explicit Init_JointCommand_position(::interface_protocol::msg::JointCommand & msg)
  : msg_(msg)
  {}
  Init_JointCommand_velocity position(::interface_protocol::msg::JointCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_JointCommand_velocity(msg_);
  }

private:
  ::interface_protocol::msg::JointCommand msg_;
};

class Init_JointCommand_header
{
public:
  Init_JointCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointCommand_position header(::interface_protocol::msg::JointCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_JointCommand_position(msg_);
  }

private:
  ::interface_protocol::msg::JointCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::JointCommand>()
{
  return interface_protocol::msg::builder::Init_JointCommand_header();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_
