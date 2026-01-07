// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/MotorDebug.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/motor_debug__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{

namespace builder
{

class Init_MotorDebug_motor_temperature
{
public:
  explicit Init_MotorDebug_motor_temperature(::interface_protocol::msg::MotorDebug & msg)
  : msg_(msg)
  {}
  ::interface_protocol::msg::MotorDebug motor_temperature(::interface_protocol::msg::MotorDebug::_motor_temperature_type arg)
  {
    msg_.motor_temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::msg::MotorDebug msg_;
};

class Init_MotorDebug_mos_temperature
{
public:
  explicit Init_MotorDebug_mos_temperature(::interface_protocol::msg::MotorDebug & msg)
  : msg_(msg)
  {}
  Init_MotorDebug_motor_temperature mos_temperature(::interface_protocol::msg::MotorDebug::_mos_temperature_type arg)
  {
    msg_.mos_temperature = std::move(arg);
    return Init_MotorDebug_motor_temperature(msg_);
  }

private:
  ::interface_protocol::msg::MotorDebug msg_;
};

class Init_MotorDebug_tau_cmd
{
public:
  Init_MotorDebug_tau_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorDebug_mos_temperature tau_cmd(::interface_protocol::msg::MotorDebug::_tau_cmd_type arg)
  {
    msg_.tau_cmd = std::move(arg);
    return Init_MotorDebug_mos_temperature(msg_);
  }

private:
  ::interface_protocol::msg::MotorDebug msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::MotorDebug>()
{
  return interface_protocol::msg::builder::Init_MotorDebug_tau_cmd();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__BUILDER_HPP_
