// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface_protocol:msg/MotorDebug.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__STRUCT_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interface_protocol__msg__MotorDebug __attribute__((deprecated))
#else
# define DEPRECATED__interface_protocol__msg__MotorDebug __declspec(deprecated)
#endif

namespace interface_protocol
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorDebug_
{
  using Type = MotorDebug_<ContainerAllocator>;

  explicit MotorDebug_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MotorDebug_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _tau_cmd_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _tau_cmd_type tau_cmd;
  using _mos_temperature_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _mos_temperature_type mos_temperature;
  using _motor_temperature_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _motor_temperature_type motor_temperature;

  // setters for named parameter idiom
  Type & set__tau_cmd(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->tau_cmd = _arg;
    return *this;
  }
  Type & set__mos_temperature(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->mos_temperature = _arg;
    return *this;
  }
  Type & set__motor_temperature(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->motor_temperature = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_protocol::msg::MotorDebug_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_protocol::msg::MotorDebug_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_protocol::msg::MotorDebug_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_protocol::msg::MotorDebug_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_protocol::msg::MotorDebug_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::msg::MotorDebug_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_protocol::msg::MotorDebug_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::msg::MotorDebug_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_protocol::msg::MotorDebug_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_protocol::msg::MotorDebug_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_protocol__msg__MotorDebug
    std::shared_ptr<interface_protocol::msg::MotorDebug_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_protocol__msg__MotorDebug
    std::shared_ptr<interface_protocol::msg::MotorDebug_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorDebug_ & other) const
  {
    if (this->tau_cmd != other.tau_cmd) {
      return false;
    }
    if (this->mos_temperature != other.mos_temperature) {
      return false;
    }
    if (this->motor_temperature != other.motor_temperature) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorDebug_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorDebug_

// alias to use template instance with default allocator
using MotorDebug =
  interface_protocol::msg::MotorDebug_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__MOTOR_DEBUG__STRUCT_HPP_
