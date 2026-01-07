// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/ParallelParserType.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__PARALLEL_PARSER_TYPE__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__PARALLEL_PARSER_TYPE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/parallel_parser_type__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{


}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::ParallelParserType>()
{
  return ::interface_protocol::msg::ParallelParserType(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__PARALLEL_PARSER_TYPE__BUILDER_HPP_
