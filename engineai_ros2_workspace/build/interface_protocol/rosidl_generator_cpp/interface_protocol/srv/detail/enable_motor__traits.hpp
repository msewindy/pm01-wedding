// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_protocol:srv/EnableMotor.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__TRAITS_HPP_
#define INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_protocol/srv/detail/enable_motor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interface_protocol
{

namespace srv
{

inline void to_flow_style_yaml(
  const EnableMotor_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: enable
  {
    out << "enable: ";
    rosidl_generator_traits::value_to_yaml(msg.enable, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EnableMotor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: enable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable: ";
    rosidl_generator_traits::value_to_yaml(msg.enable, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EnableMotor_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interface_protocol

namespace rosidl_generator_traits
{

[[deprecated("use interface_protocol::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interface_protocol::srv::EnableMotor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_protocol::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_protocol::srv::to_yaml() instead")]]
inline std::string to_yaml(const interface_protocol::srv::EnableMotor_Request & msg)
{
  return interface_protocol::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interface_protocol::srv::EnableMotor_Request>()
{
  return "interface_protocol::srv::EnableMotor_Request";
}

template<>
inline const char * name<interface_protocol::srv::EnableMotor_Request>()
{
  return "interface_protocol/srv/EnableMotor_Request";
}

template<>
struct has_fixed_size<interface_protocol::srv::EnableMotor_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interface_protocol::srv::EnableMotor_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interface_protocol::srv::EnableMotor_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace interface_protocol
{

namespace srv
{

inline void to_flow_style_yaml(
  const EnableMotor_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EnableMotor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EnableMotor_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interface_protocol

namespace rosidl_generator_traits
{

[[deprecated("use interface_protocol::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interface_protocol::srv::EnableMotor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_protocol::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_protocol::srv::to_yaml() instead")]]
inline std::string to_yaml(const interface_protocol::srv::EnableMotor_Response & msg)
{
  return interface_protocol::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interface_protocol::srv::EnableMotor_Response>()
{
  return "interface_protocol::srv::EnableMotor_Response";
}

template<>
inline const char * name<interface_protocol::srv::EnableMotor_Response>()
{
  return "interface_protocol/srv/EnableMotor_Response";
}

template<>
struct has_fixed_size<interface_protocol::srv::EnableMotor_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface_protocol::srv::EnableMotor_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface_protocol::srv::EnableMotor_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interface_protocol::srv::EnableMotor>()
{
  return "interface_protocol::srv::EnableMotor";
}

template<>
inline const char * name<interface_protocol::srv::EnableMotor>()
{
  return "interface_protocol/srv/EnableMotor";
}

template<>
struct has_fixed_size<interface_protocol::srv::EnableMotor>
  : std::integral_constant<
    bool,
    has_fixed_size<interface_protocol::srv::EnableMotor_Request>::value &&
    has_fixed_size<interface_protocol::srv::EnableMotor_Response>::value
  >
{
};

template<>
struct has_bounded_size<interface_protocol::srv::EnableMotor>
  : std::integral_constant<
    bool,
    has_bounded_size<interface_protocol::srv::EnableMotor_Request>::value &&
    has_bounded_size<interface_protocol::srv::EnableMotor_Response>::value
  >
{
};

template<>
struct is_service<interface_protocol::srv::EnableMotor>
  : std::true_type
{
};

template<>
struct is_service_request<interface_protocol::srv::EnableMotor_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interface_protocol::srv::EnableMotor_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__TRAITS_HPP_
