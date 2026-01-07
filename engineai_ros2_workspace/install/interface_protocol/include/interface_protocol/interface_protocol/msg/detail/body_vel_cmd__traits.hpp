// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_protocol:msg/BodyVelCmd.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__BODY_VEL_CMD__TRAITS_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__BODY_VEL_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_protocol/msg/detail/body_vel_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace interface_protocol
{

namespace msg
{

inline void to_flow_style_yaml(
  const BodyVelCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: linear_velocity
  {
    if (msg.linear_velocity.size() == 0) {
      out << "linear_velocity: []";
    } else {
      out << "linear_velocity: [";
      size_t pending_items = msg.linear_velocity.size();
      for (auto item : msg.linear_velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: yaw_velocity
  {
    out << "yaw_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BodyVelCmd & msg,
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

  // member: linear_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.linear_velocity.size() == 0) {
      out << "linear_velocity: []\n";
    } else {
      out << "linear_velocity:\n";
      for (auto item : msg.linear_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: yaw_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_velocity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BodyVelCmd & msg, bool use_flow_style = false)
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
  const interface_protocol::msg::BodyVelCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_protocol::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_protocol::msg::to_yaml() instead")]]
inline std::string to_yaml(const interface_protocol::msg::BodyVelCmd & msg)
{
  return interface_protocol::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interface_protocol::msg::BodyVelCmd>()
{
  return "interface_protocol::msg::BodyVelCmd";
}

template<>
inline const char * name<interface_protocol::msg::BodyVelCmd>()
{
  return "interface_protocol/msg/BodyVelCmd";
}

template<>
struct has_fixed_size<interface_protocol::msg::BodyVelCmd>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface_protocol::msg::BodyVelCmd>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface_protocol::msg::BodyVelCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__BODY_VEL_CMD__TRAITS_HPP_
