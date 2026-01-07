// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_protocol:msg/ImuInfo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__TRAITS_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_protocol/msg/detail/imu_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"
// Member 'rpy'
// Member 'linear_acceleration'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace interface_protocol
{

namespace msg
{

inline void to_flow_style_yaml(
  const ImuInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: quaternion
  {
    out << "quaternion: ";
    to_flow_style_yaml(msg.quaternion, out);
    out << ", ";
  }

  // member: rpy
  {
    out << "rpy: ";
    to_flow_style_yaml(msg.rpy, out);
    out << ", ";
  }

  // member: linear_acceleration
  {
    out << "linear_acceleration: ";
    to_flow_style_yaml(msg.linear_acceleration, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    to_flow_style_yaml(msg.angular_velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImuInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: quaternion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quaternion:\n";
    to_block_style_yaml(msg.quaternion, out, indentation + 2);
  }

  // member: rpy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rpy:\n";
    to_block_style_yaml(msg.rpy, out, indentation + 2);
  }

  // member: linear_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_acceleration:\n";
    to_block_style_yaml(msg.linear_acceleration, out, indentation + 2);
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity:\n";
    to_block_style_yaml(msg.angular_velocity, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImuInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace interface_protocol

namespace rosidl_generator_traits
{

[[deprecated("use interface_protocol::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interface_protocol::msg::ImuInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_protocol::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_protocol::msg::to_yaml() instead")]]
inline std::string to_yaml(const interface_protocol::msg::ImuInfo & msg)
{
  return interface_protocol::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interface_protocol::msg::ImuInfo>()
{
  return "interface_protocol::msg::ImuInfo";
}

template<>
inline const char * name<interface_protocol::msg::ImuInfo>()
{
  return "interface_protocol/msg/ImuInfo";
}

template<>
struct has_fixed_size<interface_protocol::msg::ImuInfo>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<interface_protocol::msg::ImuInfo>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<interface_protocol::msg::ImuInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__TRAITS_HPP_
