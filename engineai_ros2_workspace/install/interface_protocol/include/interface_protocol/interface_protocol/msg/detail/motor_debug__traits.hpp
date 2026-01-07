// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_protocol:msg/MotorDebug.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__TRAITS_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_protocol/msg/detail/motor_debug__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interface_protocol
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorDebug & msg,
  std::ostream & out)
{
  out << "{";
  // member: tau_cmd
  {
    if (msg.tau_cmd.size() == 0) {
      out << "tau_cmd: []";
    } else {
      out << "tau_cmd: [";
      size_t pending_items = msg.tau_cmd.size();
      for (auto item : msg.tau_cmd) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: mos_temperature
  {
    if (msg.mos_temperature.size() == 0) {
      out << "mos_temperature: []";
    } else {
      out << "mos_temperature: [";
      size_t pending_items = msg.mos_temperature.size();
      for (auto item : msg.mos_temperature) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: motor_temperature
  {
    if (msg.motor_temperature.size() == 0) {
      out << "motor_temperature: []";
    } else {
      out << "motor_temperature: [";
      size_t pending_items = msg.motor_temperature.size();
      for (auto item : msg.motor_temperature) {
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
  const MotorDebug & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: tau_cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tau_cmd.size() == 0) {
      out << "tau_cmd: []\n";
    } else {
      out << "tau_cmd:\n";
      for (auto item : msg.tau_cmd) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: mos_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.mos_temperature.size() == 0) {
      out << "mos_temperature: []\n";
    } else {
      out << "mos_temperature:\n";
      for (auto item : msg.mos_temperature) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: motor_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.motor_temperature.size() == 0) {
      out << "motor_temperature: []\n";
    } else {
      out << "motor_temperature:\n";
      for (auto item : msg.motor_temperature) {
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

inline std::string to_yaml(const MotorDebug & msg, bool use_flow_style = false)
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
  const interface_protocol::msg::MotorDebug & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_protocol::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_protocol::msg::to_yaml() instead")]]
inline std::string to_yaml(const interface_protocol::msg::MotorDebug & msg)
{
  return interface_protocol::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interface_protocol::msg::MotorDebug>()
{
  return "interface_protocol::msg::MotorDebug";
}

template<>
inline const char * name<interface_protocol::msg::MotorDebug>()
{
  return "interface_protocol/msg/MotorDebug";
}

template<>
struct has_fixed_size<interface_protocol::msg::MotorDebug>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface_protocol::msg::MotorDebug>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface_protocol::msg::MotorDebug>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__TRAITS_HPP_
