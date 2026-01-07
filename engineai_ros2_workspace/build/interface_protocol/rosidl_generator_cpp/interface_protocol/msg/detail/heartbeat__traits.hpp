// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_protocol:msg/Heartbeat.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__HEARTBEAT__TRAITS_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__HEARTBEAT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_protocol/msg/detail/heartbeat__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace interface_protocol
{

namespace msg
{

inline void to_flow_style_yaml(
  const Heartbeat & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: node_name
  {
    out << "node_name: ";
    rosidl_generator_traits::value_to_yaml(msg.node_name, out);
    out << ", ";
  }

  // member: startup_timestamp
  {
    out << "startup_timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.startup_timestamp, out);
    out << ", ";
  }

  // member: node_status
  {
    out << "node_status: ";
    rosidl_generator_traits::value_to_yaml(msg.node_status, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: error_message
  {
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Heartbeat & msg,
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

  // member: node_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "node_name: ";
    rosidl_generator_traits::value_to_yaml(msg.node_name, out);
    out << "\n";
  }

  // member: startup_timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "startup_timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.startup_timestamp, out);
    out << "\n";
  }

  // member: node_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "node_status: ";
    rosidl_generator_traits::value_to_yaml(msg.node_status, out);
    out << "\n";
  }

  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << "\n";
  }

  // member: error_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_message: ";
    rosidl_generator_traits::value_to_yaml(msg.error_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Heartbeat & msg, bool use_flow_style = false)
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
  const interface_protocol::msg::Heartbeat & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_protocol::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_protocol::msg::to_yaml() instead")]]
inline std::string to_yaml(const interface_protocol::msg::Heartbeat & msg)
{
  return interface_protocol::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interface_protocol::msg::Heartbeat>()
{
  return "interface_protocol::msg::Heartbeat";
}

template<>
inline const char * name<interface_protocol::msg::Heartbeat>()
{
  return "interface_protocol/msg/Heartbeat";
}

template<>
struct has_fixed_size<interface_protocol::msg::Heartbeat>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface_protocol::msg::Heartbeat>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface_protocol::msg::Heartbeat>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__HEARTBEAT__TRAITS_HPP_
