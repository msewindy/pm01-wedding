// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/BodyVelCmd.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__BODY_VEL_CMD__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__BODY_VEL_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/body_vel_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{

namespace builder
{

class Init_BodyVelCmd_yaw_velocity
{
public:
  explicit Init_BodyVelCmd_yaw_velocity(::interface_protocol::msg::BodyVelCmd & msg)
  : msg_(msg)
  {}
  ::interface_protocol::msg::BodyVelCmd yaw_velocity(::interface_protocol::msg::BodyVelCmd::_yaw_velocity_type arg)
  {
    msg_.yaw_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::msg::BodyVelCmd msg_;
};

class Init_BodyVelCmd_linear_velocity
{
public:
  explicit Init_BodyVelCmd_linear_velocity(::interface_protocol::msg::BodyVelCmd & msg)
  : msg_(msg)
  {}
  Init_BodyVelCmd_yaw_velocity linear_velocity(::interface_protocol::msg::BodyVelCmd::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_BodyVelCmd_yaw_velocity(msg_);
  }

private:
  ::interface_protocol::msg::BodyVelCmd msg_;
};

class Init_BodyVelCmd_header
{
public:
  Init_BodyVelCmd_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BodyVelCmd_linear_velocity header(::interface_protocol::msg::BodyVelCmd::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BodyVelCmd_linear_velocity(msg_);
  }

private:
  ::interface_protocol::msg::BodyVelCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::BodyVelCmd>()
{
  return interface_protocol::msg::builder::Init_BodyVelCmd_header();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__BODY_VEL_CMD__BUILDER_HPP_
