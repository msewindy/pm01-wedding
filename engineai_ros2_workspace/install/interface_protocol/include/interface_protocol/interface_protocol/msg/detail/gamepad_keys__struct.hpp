// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface_protocol:msg/GamepadKeys.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__STRUCT_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface_protocol__msg__GamepadKeys __attribute__((deprecated))
#else
# define DEPRECATED__interface_protocol__msg__GamepadKeys __declspec(deprecated)
#endif

namespace interface_protocol
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GamepadKeys_
{
  using Type = GamepadKeys_<ContainerAllocator>;

  explicit GamepadKeys_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int32_t, 12>::iterator, int32_t>(this->digital_states.begin(), this->digital_states.end(), 0l);
      std::fill<typename std::array<double, 6>::iterator, double>(this->analog_states.begin(), this->analog_states.end(), 0.0);
    }
  }

  explicit GamepadKeys_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    digital_states(_alloc),
    analog_states(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<int32_t, 12>::iterator, int32_t>(this->digital_states.begin(), this->digital_states.end(), 0l);
      std::fill<typename std::array<double, 6>::iterator, double>(this->analog_states.begin(), this->analog_states.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _digital_states_type =
    std::array<int32_t, 12>;
  _digital_states_type digital_states;
  using _analog_states_type =
    std::array<double, 6>;
  _analog_states_type analog_states;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__digital_states(
    const std::array<int32_t, 12> & _arg)
  {
    this->digital_states = _arg;
    return *this;
  }
  Type & set__analog_states(
    const std::array<double, 6> & _arg)
  {
    this->analog_states = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t LB =
    0u;
  static constexpr uint8_t RB =
    1u;
  static constexpr uint8_t A =
    2u;
  static constexpr uint8_t B =
    3u;
  static constexpr uint8_t X =
    4u;
  static constexpr uint8_t Y =
    5u;
  static constexpr uint8_t BACK =
    6u;
  static constexpr uint8_t START =
    7u;
  static constexpr uint8_t CROSS_X_UP =
    8u;
  static constexpr uint8_t CROSS_X_DOWN =
    9u;
  static constexpr uint8_t CROSS_Y_LEFT =
    10u;
  static constexpr uint8_t CROSS_Y_RIGHT =
    11u;
  static constexpr uint8_t LT =
    0u;
  static constexpr uint8_t RT =
    1u;
  static constexpr uint8_t LEFT_STICK_X =
    2u;
  static constexpr uint8_t LEFT_STICK_Y =
    3u;
  static constexpr uint8_t RIGHT_STICK_X =
    4u;
  static constexpr uint8_t RIGHT_STICK_Y =
    5u;

  // pointer types
  using RawPtr =
    interface_protocol::msg::GamepadKeys_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_protocol::msg::GamepadKeys_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_protocol::msg::GamepadKeys_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_protocol::msg::GamepadKeys_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_protocol::msg::GamepadKeys_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::msg::GamepadKeys_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_protocol::msg::GamepadKeys_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::msg::GamepadKeys_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_protocol::msg::GamepadKeys_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_protocol::msg::GamepadKeys_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_protocol__msg__GamepadKeys
    std::shared_ptr<interface_protocol::msg::GamepadKeys_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_protocol__msg__GamepadKeys
    std::shared_ptr<interface_protocol::msg::GamepadKeys_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GamepadKeys_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->digital_states != other.digital_states) {
      return false;
    }
    if (this->analog_states != other.analog_states) {
      return false;
    }
    return true;
  }
  bool operator!=(const GamepadKeys_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GamepadKeys_

// alias to use template instance with default allocator
using GamepadKeys =
  interface_protocol::msg::GamepadKeys_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::LB;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::RB;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::A;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::B;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::X;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::Y;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::BACK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::CROSS_X_UP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::CROSS_X_DOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::CROSS_Y_LEFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::CROSS_Y_RIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::LT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::RT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::LEFT_STICK_X;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::LEFT_STICK_Y;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::RIGHT_STICK_X;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t GamepadKeys_<ContainerAllocator>::RIGHT_STICK_Y;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__GAMEPAD_KEYS__STRUCT_HPP_
