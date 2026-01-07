// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_protocol:msg/ParallelParserType.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__PARALLEL_PARSER_TYPE__TRAITS_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__PARALLEL_PARSER_TYPE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_protocol/msg/detail/parallel_parser_type__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interface_protocol
{

namespace msg
{

inline void to_flow_style_yaml(
  const ParallelParserType & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParallelParserType & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParallelParserType & msg, bool use_flow_style = false)
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
  const interface_protocol::msg::ParallelParserType & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_protocol::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_protocol::msg::to_yaml() instead")]]
inline std::string to_yaml(const interface_protocol::msg::ParallelParserType & msg)
{
  return interface_protocol::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interface_protocol::msg::ParallelParserType>()
{
  return "interface_protocol::msg::ParallelParserType";
}

template<>
inline const char * name<interface_protocol::msg::ParallelParserType>()
{
  return "interface_protocol/msg/ParallelParserType";
}

template<>
struct has_fixed_size<interface_protocol::msg::ParallelParserType>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface_protocol::msg::ParallelParserType>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface_protocol::msg::ParallelParserType>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__PARALLEL_PARSER_TYPE__TRAITS_HPP_
