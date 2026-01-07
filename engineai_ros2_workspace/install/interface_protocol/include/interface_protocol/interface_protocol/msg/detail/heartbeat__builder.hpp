// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/Heartbeat.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__HEARTBEAT__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__HEARTBEAT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/heartbeat__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{

namespace builder
{

class Init_Heartbeat_error_message
{
public:
  explicit Init_Heartbeat_error_message(::interface_protocol::msg::Heartbeat & msg)
  : msg_(msg)
  {}
  ::interface_protocol::msg::Heartbeat error_message(::interface_protocol::msg::Heartbeat::_error_message_type arg)
  {
    msg_.error_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::msg::Heartbeat msg_;
};

class Init_Heartbeat_error_code
{
public:
  explicit Init_Heartbeat_error_code(::interface_protocol::msg::Heartbeat & msg)
  : msg_(msg)
  {}
  Init_Heartbeat_error_message error_code(::interface_protocol::msg::Heartbeat::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_Heartbeat_error_message(msg_);
  }

private:
  ::interface_protocol::msg::Heartbeat msg_;
};

class Init_Heartbeat_node_status
{
public:
  explicit Init_Heartbeat_node_status(::interface_protocol::msg::Heartbeat & msg)
  : msg_(msg)
  {}
  Init_Heartbeat_error_code node_status(::interface_protocol::msg::Heartbeat::_node_status_type arg)
  {
    msg_.node_status = std::move(arg);
    return Init_Heartbeat_error_code(msg_);
  }

private:
  ::interface_protocol::msg::Heartbeat msg_;
};

class Init_Heartbeat_startup_timestamp
{
public:
  explicit Init_Heartbeat_startup_timestamp(::interface_protocol::msg::Heartbeat & msg)
  : msg_(msg)
  {}
  Init_Heartbeat_node_status startup_timestamp(::interface_protocol::msg::Heartbeat::_startup_timestamp_type arg)
  {
    msg_.startup_timestamp = std::move(arg);
    return Init_Heartbeat_node_status(msg_);
  }

private:
  ::interface_protocol::msg::Heartbeat msg_;
};

class Init_Heartbeat_node_name
{
public:
  explicit Init_Heartbeat_node_name(::interface_protocol::msg::Heartbeat & msg)
  : msg_(msg)
  {}
  Init_Heartbeat_startup_timestamp node_name(::interface_protocol::msg::Heartbeat::_node_name_type arg)
  {
    msg_.node_name = std::move(arg);
    return Init_Heartbeat_startup_timestamp(msg_);
  }

private:
  ::interface_protocol::msg::Heartbeat msg_;
};

class Init_Heartbeat_header
{
public:
  Init_Heartbeat_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Heartbeat_node_name header(::interface_protocol::msg::Heartbeat::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Heartbeat_node_name(msg_);
  }

private:
  ::interface_protocol::msg::Heartbeat msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::Heartbeat>()
{
  return interface_protocol::msg::builder::Init_Heartbeat_header();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__HEARTBEAT__BUILDER_HPP_
