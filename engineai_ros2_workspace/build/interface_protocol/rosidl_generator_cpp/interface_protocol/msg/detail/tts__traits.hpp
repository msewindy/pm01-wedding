// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_protocol:msg/Tts.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__TTS__TRAITS_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__TTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_protocol/msg/detail/tts__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interface_protocol
{

namespace msg
{

inline void to_flow_style_yaml(
  const Tts & msg,
  std::ostream & out)
{
  out << "{";
  // member: text
  {
    out << "text: ";
    rosidl_generator_traits::value_to_yaml(msg.text, out);
    out << ", ";
  }

  // member: language
  {
    out << "language: ";
    rosidl_generator_traits::value_to_yaml(msg.language, out);
    out << ", ";
  }

  // member: speaker
  {
    out << "speaker: ";
    rosidl_generator_traits::value_to_yaml(msg.speaker, out);
    out << ", ";
  }

  // member: rate
  {
    out << "rate: ";
    rosidl_generator_traits::value_to_yaml(msg.rate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Tts & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: text
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "text: ";
    rosidl_generator_traits::value_to_yaml(msg.text, out);
    out << "\n";
  }

  // member: language
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "language: ";
    rosidl_generator_traits::value_to_yaml(msg.language, out);
    out << "\n";
  }

  // member: speaker
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speaker: ";
    rosidl_generator_traits::value_to_yaml(msg.speaker, out);
    out << "\n";
  }

  // member: rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rate: ";
    rosidl_generator_traits::value_to_yaml(msg.rate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Tts & msg, bool use_flow_style = false)
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
  const interface_protocol::msg::Tts & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_protocol::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_protocol::msg::to_yaml() instead")]]
inline std::string to_yaml(const interface_protocol::msg::Tts & msg)
{
  return interface_protocol::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interface_protocol::msg::Tts>()
{
  return "interface_protocol::msg::Tts";
}

template<>
inline const char * name<interface_protocol::msg::Tts>()
{
  return "interface_protocol/msg/Tts";
}

template<>
struct has_fixed_size<interface_protocol::msg::Tts>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface_protocol::msg::Tts>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface_protocol::msg::Tts>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__TTS__TRAITS_HPP_
