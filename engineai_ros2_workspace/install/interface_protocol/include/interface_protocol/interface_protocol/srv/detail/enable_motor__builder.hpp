// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:srv/EnableMotor.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__BUILDER_HPP_
#define INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/srv/detail/enable_motor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace srv
{

namespace builder
{

class Init_EnableMotor_Request_enable
{
public:
  Init_EnableMotor_Request_enable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface_protocol::srv::EnableMotor_Request enable(::interface_protocol::srv::EnableMotor_Request::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::srv::EnableMotor_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::srv::EnableMotor_Request>()
{
  return interface_protocol::srv::builder::Init_EnableMotor_Request_enable();
}

}  // namespace interface_protocol


namespace interface_protocol
{

namespace srv
{

namespace builder
{

class Init_EnableMotor_Response_message
{
public:
  explicit Init_EnableMotor_Response_message(::interface_protocol::srv::EnableMotor_Response & msg)
  : msg_(msg)
  {}
  ::interface_protocol::srv::EnableMotor_Response message(::interface_protocol::srv::EnableMotor_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::srv::EnableMotor_Response msg_;
};

class Init_EnableMotor_Response_success
{
public:
  Init_EnableMotor_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EnableMotor_Response_message success(::interface_protocol::srv::EnableMotor_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_EnableMotor_Response_message(msg_);
  }

private:
  ::interface_protocol::srv::EnableMotor_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::srv::EnableMotor_Response>()
{
  return interface_protocol::srv::builder::Init_EnableMotor_Response_success();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__BUILDER_HPP_
