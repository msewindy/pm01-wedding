// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from interface_protocol:msg/MotorDebug.idl
// generated code does not contain a copyright notice
#include "interface_protocol/msg/detail/motor_debug__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "interface_protocol/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interface_protocol/msg/detail/motor_debug__struct.h"
#include "interface_protocol/msg/detail/motor_debug__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // mos_temperature, motor_temperature, tau_cmd
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // mos_temperature, motor_temperature, tau_cmd

// forward declare type support functions


using _MotorDebug__ros_msg_type = interface_protocol__msg__MotorDebug;

static bool _MotorDebug__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MotorDebug__ros_msg_type * ros_message = static_cast<const _MotorDebug__ros_msg_type *>(untyped_ros_message);
  // Field name: tau_cmd
  {
    size_t size = ros_message->tau_cmd.size;
    auto array_ptr = ros_message->tau_cmd.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: mos_temperature
  {
    size_t size = ros_message->mos_temperature.size;
    auto array_ptr = ros_message->mos_temperature.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: motor_temperature
  {
    size_t size = ros_message->motor_temperature.size;
    auto array_ptr = ros_message->motor_temperature.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _MotorDebug__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MotorDebug__ros_msg_type * ros_message = static_cast<_MotorDebug__ros_msg_type *>(untyped_ros_message);
  // Field name: tau_cmd
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->tau_cmd.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->tau_cmd);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->tau_cmd, size)) {
      fprintf(stderr, "failed to create array for field 'tau_cmd'");
      return false;
    }
    auto array_ptr = ros_message->tau_cmd.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: mos_temperature
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->mos_temperature.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->mos_temperature);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->mos_temperature, size)) {
      fprintf(stderr, "failed to create array for field 'mos_temperature'");
      return false;
    }
    auto array_ptr = ros_message->mos_temperature.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: motor_temperature
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.getState();
    bool correct_size = cdr.jump(size);
    cdr.setState(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->motor_temperature.data) {
      rosidl_runtime_c__double__Sequence__fini(&ros_message->motor_temperature);
    }
    if (!rosidl_runtime_c__double__Sequence__init(&ros_message->motor_temperature, size)) {
      fprintf(stderr, "failed to create array for field 'motor_temperature'");
      return false;
    }
    auto array_ptr = ros_message->motor_temperature.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_protocol
size_t get_serialized_size_interface_protocol__msg__MotorDebug(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MotorDebug__ros_msg_type * ros_message = static_cast<const _MotorDebug__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name tau_cmd
  {
    size_t array_size = ros_message->tau_cmd.size;
    auto array_ptr = ros_message->tau_cmd.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mos_temperature
  {
    size_t array_size = ros_message->mos_temperature.size;
    auto array_ptr = ros_message->mos_temperature.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor_temperature
  {
    size_t array_size = ros_message->motor_temperature.size;
    auto array_ptr = ros_message->motor_temperature.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MotorDebug__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_interface_protocol__msg__MotorDebug(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface_protocol
size_t max_serialized_size_interface_protocol__msg__MotorDebug(
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

  // member: tau_cmd
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: mos_temperature
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: motor_temperature
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = interface_protocol__msg__MotorDebug;
    is_plain =
      (
      offsetof(DataType, motor_temperature) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MotorDebug__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_interface_protocol__msg__MotorDebug(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MotorDebug = {
  "interface_protocol::msg",
  "MotorDebug",
  _MotorDebug__cdr_serialize,
  _MotorDebug__cdr_deserialize,
  _MotorDebug__get_serialized_size,
  _MotorDebug__max_serialized_size
};

static rosidl_message_type_support_t _MotorDebug__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MotorDebug,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface_protocol, msg, MotorDebug)() {
  return &_MotorDebug__type_support;
}

#if defined(__cplusplus)
}
#endif
