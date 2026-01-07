// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/MotionState.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__MOTION_STATE__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__MOTION_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/motion_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{

namespace builder
{

class Init_MotionState_current_motion_task
{
public:
  Init_MotionState_current_motion_task()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface_protocol::msg::MotionState current_motion_task(::interface_protocol::msg::MotionState::_current_motion_task_type arg)
  {
    msg_.current_motion_task = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::msg::MotionState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::MotionState>()
{
  return interface_protocol::msg::builder::Init_MotionState_current_motion_task();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__MOTION_STATE__BUILDER_HPP_
