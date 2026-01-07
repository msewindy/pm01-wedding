// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from interface_protocol:msg/ParallelParserType.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "interface_protocol/msg/detail/parallel_parser_type__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace interface_protocol
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ParallelParserType_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) interface_protocol::msg::ParallelParserType(_init);
}

void ParallelParserType_fini_function(void * message_memory)
{
  auto typed_message = static_cast<interface_protocol::msg::ParallelParserType *>(message_memory);
  typed_message->~ParallelParserType();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ParallelParserType_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interface_protocol::msg::ParallelParserType, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ParallelParserType_message_members = {
  "interface_protocol::msg",  // message namespace
  "ParallelParserType",  // message name
  1,  // number of fields
  sizeof(interface_protocol::msg::ParallelParserType),
  ParallelParserType_message_member_array,  // message members
  ParallelParserType_init_function,  // function to initialize message memory (memory has to be allocated)
  ParallelParserType_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ParallelParserType_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ParallelParserType_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace interface_protocol


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<interface_protocol::msg::ParallelParserType>()
{
  return &::interface_protocol::msg::rosidl_typesupport_introspection_cpp::ParallelParserType_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, interface_protocol, msg, ParallelParserType)() {
  return &::interface_protocol::msg::rosidl_typesupport_introspection_cpp::ParallelParserType_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
