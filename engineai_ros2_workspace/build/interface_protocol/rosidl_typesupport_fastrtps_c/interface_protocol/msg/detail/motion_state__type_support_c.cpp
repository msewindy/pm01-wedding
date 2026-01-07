// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from interface_protocol:msg/MotionState.idl
// generated code does not contain a copyright notice
#include "interface_protocol/msg/detail/motion_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "interface_protocol/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interface_protocol/msg/detail/motion_state__struct.h"
#include "interface_protocol/msg/detail/motion_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // current_motion_task
#include "rosidl_runtime_c/string_functions.h"  // current_motion_task

// forward declare type support functions


using _MotionState__ros_msg_type = interface_protocol__msg__MotionState;

static bool _MotionState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MotionState__ros_msg_type * ros_message = static_cast<const _MotionState__ros_msg_type *>(untyped_ros_message);
  // Field name: current_motion_task
  {
    const rosidl_runtime_c__String * str = &ros_message->current_motion_task;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _MotionState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MotionState__ros_msg_type * ros_message = static_cast<_MotionState__ros_msg_type *>(untyped_ros_message);
  // Field name: current_motion_task
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->current_motion_task.data) {
      rosidl_runtime_c__String__init(&ros_message->current_motion_task);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->current_motion_task,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'current_motion_task'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_protocol
size_t get_serialized_size_interface_protocol__msg__MotionState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MotionState__ros_msg_type * ros_message = static_cast<const _MotionState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name current_motion_task
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->current_motion_task.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _MotionState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_interface_protocol__msg__MotionState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_protocol
size_t max_serialized_size_interface_protocol__msg__MotionState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: current_motion_task
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = interface_protocol__msg__MotionState;
    is_plain =
      (
      offsetof(DataType, current_motion_task) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MotionState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_interface_protocol__msg__MotionState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MotionState = {
  "interface_protocol::msg",
  "MotionState",
  _MotionState__cdr_serialize,
  _MotionState__cdr_deserialize,
  _MotionState__get_serialized_size,
  _MotionState__max_serialized_size
};

static rosidl_message_type_support_t _MotionState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MotionState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface_protocol, msg, MotionState)() {
  return &_MotionState__type_support;
}

#if defined(__cplusplus)
}
#endif
