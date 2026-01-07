// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/JointState.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/joint_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{

namespace builder
{

class Init_JointState_torque
{
public:
  explicit Init_JointState_torque(::interface_protocol::msg::JointState & msg)
  : msg_(msg)
  {}
  ::interface_protocol::msg::JointState torque(::interface_protocol::msg::JointState::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::msg::JointState msg_;
};

class Init_JointState_velocity
{
public:
  explicit Init_JointState_velocity(::interface_protocol::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_torque velocity(::interface_protocol::msg::JointState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_JointState_torque(msg_);
  }

private:
  ::interface_protocol::msg::JointState msg_;
};

class Init_JointState_position
{
public:
  explicit Init_JointState_position(::interface_protocol::msg::JointState & msg)
  : msg_(msg)
  {}
  Init_JointState_velocity position(::interface_protocol::msg::JointState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_JointState_velocity(msg_);
  }

private:
  ::interface_protocol::msg::JointState msg_;
};

class Init_JointState_header
{
public:
  Init_JointState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointState_position header(::interface_protocol::msg::JointState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_JointState_position(msg_);
  }

private:
  ::interface_protocol::msg::JointState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::JointState>()
{
  return interface_protocol::msg::builder::Init_JointState_header();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__JOINT_STATE__BUILDER_HPP_
