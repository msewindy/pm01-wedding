// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from interface_protocol:msg/ParallelParserType.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "interface_protocol/msg/detail/parallel_parser_type__struct.h"
#include "interface_protocol/msg/detail/parallel_parser_type__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace interface_protocol
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _ParallelParserType_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ParallelParserType_type_support_ids_t;

static const _ParallelParserType_type_support_ids_t _ParallelParserType_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _ParallelParserType_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ParallelParserType_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ParallelParserType_type_support_symbol_names_t _ParallelParserType_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface_protocol, msg, ParallelParserType)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interface_protocol, msg, ParallelParserType)),
  }
};

typedef struct _ParallelParserType_type_support_data_t
{
  void * data[2];
} _ParallelParserType_type_support_data_t;

static _ParallelParserType_type_support_data_t _ParallelParserType_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ParallelParserType_message_typesupport_map = {
  2,
  "interface_protocol",
  &_ParallelParserType_message_typesupport_ids.typesupport_identifier[0],
  &_ParallelParserType_message_typesupport_symbol_names.symbol_name[0],
  &_ParallelParserType_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ParallelParserType_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ParallelParserType_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace interface_protocol

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, interface_protocol, msg, ParallelParserType)() {
  return &::interface_protocol::msg::rosidl_typesupport_c::ParallelParserType_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
