// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interface_protocol:msg/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interface_protocol/msg/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace interface_protocol
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: velocity
  {
    if (msg.velocity.size() == 0) {
      out << "velocity: []";
    } else {
      out << "velocity: [";
      size_t pending_items = msg.velocity.size();
      for (auto item : msg.velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: feed_forward_torque
  {
    if (msg.feed_forward_torque.size() == 0) {
      out << "feed_forward_torque: []";
    } else {
      out << "feed_forward_torque: [";
      size_t pending_items = msg.feed_forward_torque.size();
      for (auto item : msg.feed_forward_torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: torque
  {
    if (msg.torque.size() == 0) {
      out << "torque: []";
    } else {
      out << "torque: [";
      size_t pending_items = msg.torque.size();
      for (auto item : msg.torque) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: stiffness
  {
    if (msg.stiffness.size() == 0) {
      out << "stiffness: []";
    } else {
      out << "stiffness: [";
      size_t pending_items = msg.stiffness.size();
      for (auto item : msg.stiffness) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: damping
  {
    if (msg.damping.size() == 0) {
      out << "damping: []";
    } else {
      out << "damping: [";
      size_t pending_items = msg.damping.size();
      for (auto item : msg.damping) {
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
  const MotorCommand & msg,
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

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.velocity.size() == 0) {
      out << "velocity: []\n";
    } else {
      out << "velocity:\n";
      for (auto item : msg.velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: feed_forward_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.feed_forward_torque.size() == 0) {
      out << "feed_forward_torque: []\n";
    } else {
      out << "feed_forward_torque:\n";
      for (auto item : msg.feed_forward_torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.torque.size() == 0) {
      out << "torque: []\n";
    } else {
      out << "torque:\n";
      for (auto item : msg.torque) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: stiffness
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.stiffness.size() == 0) {
      out << "stiffness: []\n";
    } else {
      out << "stiffness:\n";
      for (auto item : msg.stiffness) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: damping
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.damping.size() == 0) {
      out << "damping: []\n";
    } else {
      out << "damping:\n";
      for (auto item : msg.damping) {
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

inline std::string to_yaml(const MotorCommand & msg, bool use_flow_style = false)
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
  const interface_protocol::msg::MotorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  interface_protocol::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interface_protocol::msg::to_yaml() instead")]]
inline std::string to_yaml(const interface_protocol::msg::MotorCommand & msg)
{
  return interface_protocol::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interface_protocol::msg::MotorCommand>()
{
  return "interface_protocol::msg::MotorCommand";
}

template<>
inline const char * name<interface_protocol::msg::MotorCommand>()
{
  return "interface_protocol/msg/MotorCommand";
}

template<>
struct has_fixed_size<interface_protocol::msg::MotorCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interface_protocol::msg::MotorCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interface_protocol::msg::MotorCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
