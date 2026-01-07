// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_protocol:msg/GamepadKeys.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__TRAITS_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_protocol/msg/detail/gamepad_keys__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace interface_protocol
{

namespace msg
{

inline void to_flow_style_yaml(
  const GamepadKeys & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: digital_states
  {
    if (msg.digital_states.size() == 0) {
      out << "digital_states: []";
    } else {
      out << "digital_states: [";
      size_t pending_items = msg.digital_states.size();
      for (auto item : msg.digital_states) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: analog_states
  {
    if (msg.analog_states.size() == 0) {
      out << "analog_states: []";
    } else {
      out << "analog_states: [";
      size_t pending_items = msg.analog_states.size();
      for (auto item : msg.analog_states) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GamepadKeys & msg,
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

  // member: digital_states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.digital_states.size() == 0) {
      out << "digital_states: []\n";
    } else {
      out << "digital_states:\n";
      for (auto item : msg.digital_states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: analog_states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.analog_states.size() == 0) {
      out << "analog_states: []\n";
    } else {
      out << "analog_states:\n";
      for (auto item : msg.analog_states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GamepadKeys & msg, bool use_flow_style = false)
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
  const interface_protocol::msg::GamepadKeys & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_protocol::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_protocol::msg::to_yaml() instead")]]
inline std::string to_yaml(const interface_protocol::msg::GamepadKeys & msg)
{
  return interface_protocol::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interface_protocol::msg::GamepadKeys>()
{
  return "interface_protocol::msg::GamepadKeys";
}

template<>
inline const char * name<interface_protocol::msg::GamepadKeys>()
{
  return "interface_protocol/msg/GamepadKeys";
}

template<>
struct has_fixed_size<interface_protocol::msg::GamepadKeys>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<interface_protocol::msg::GamepadKeys>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<interface_protocol::msg::GamepadKeys>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__TRAITS_HPP_
