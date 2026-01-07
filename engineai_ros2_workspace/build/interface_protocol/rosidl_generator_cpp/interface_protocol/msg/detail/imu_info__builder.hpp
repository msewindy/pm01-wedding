// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/ImuInfo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/imu_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{

namespace builder
{

class Init_ImuInfo_angular_velocity
{
public:
  explicit Init_ImuInfo_angular_velocity(::interface_protocol::msg::ImuInfo & msg)
  : msg_(msg)
  {}
  ::interface_protocol::msg::ImuInfo angular_velocity(::interface_protocol::msg::ImuInfo::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::msg::ImuInfo msg_;
};

class Init_ImuInfo_linear_acceleration
{
public:
  explicit Init_ImuInfo_linear_acceleration(::interface_protocol::msg::ImuInfo & msg)
  : msg_(msg)
  {}
  Init_ImuInfo_angular_velocity linear_acceleration(::interface_protocol::msg::ImuInfo::_linear_acceleration_type arg)
  {
    msg_.linear_acceleration = std::move(arg);
    return Init_ImuInfo_angular_velocity(msg_);
  }

private:
  ::interface_protocol::msg::ImuInfo msg_;
};

class Init_ImuInfo_rpy
{
public:
  explicit Init_ImuInfo_rpy(::interface_protocol::msg::ImuInfo & msg)
  : msg_(msg)
  {}
  Init_ImuInfo_linear_acceleration rpy(::interface_protocol::msg::ImuInfo::_rpy_type arg)
  {
    msg_.rpy = std::move(arg);
    return Init_ImuInfo_linear_acceleration(msg_);
  }

private:
  ::interface_protocol::msg::ImuInfo msg_;
};

class Init_ImuInfo_quaternion
{
public:
  explicit Init_ImuInfo_quaternion(::interface_protocol::msg::ImuInfo & msg)
  : msg_(msg)
  {}
  Init_ImuInfo_rpy quaternion(::interface_protocol::msg::ImuInfo::_quaternion_type arg)
  {
    msg_.quaternion = std::move(arg);
    return Init_ImuInfo_rpy(msg_);
  }

private:
  ::interface_protocol::msg::ImuInfo msg_;
};

class Init_ImuInfo_header
{
public:
  Init_ImuInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImuInfo_quaternion header(::interface_protocol::msg::ImuInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ImuInfo_quaternion(msg_);
  }

private:
  ::interface_protocol::msg::ImuInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::ImuInfo>()
{
  return interface_protocol::msg::builder::Init_ImuInfo_header();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__BUILDER_HPP_
