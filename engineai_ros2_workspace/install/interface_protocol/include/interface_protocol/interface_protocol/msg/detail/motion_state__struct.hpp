// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface_protocol:msg/MotionState.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__MOTION_STATE__STRUCT_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__MOTION_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interface_protocol__msg__MotionState __attribute__((deprecated))
#else
# define DEPRECATED__interface_protocol__msg__MotionState __declspec(deprecated)
#endif

namespace interface_protocol
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotionState_
{
  using Type = MotionState_<ContainerAllocator>;

  explicit MotionState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_motion_task = "";
    }
  }

  explicit MotionState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_motion_task(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_motion_task = "";
    }
  }

  // field types and members
  using _current_motion_task_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _current_motion_task_type current_motion_task;

  // setters for named parameter idiom
  Type & set__current_motion_task(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->current_motion_task = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_protocol::msg::MotionState_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_protocol::msg::MotionState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_protocol::msg::MotionState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_protocol::msg::MotionState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_protocol::msg::MotionState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::msg::MotionState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_protocol::msg::MotionState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::msg::MotionState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_protocol::msg::MotionState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_protocol::msg::MotionState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_protocol__msg__MotionState
    std::shared_ptr<interface_protocol::msg::MotionState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_protocol__msg__MotionState
    std::shared_ptr<interface_protocol::msg::MotionState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotionState_ & other) const
  {
    if (this->current_motion_task != other.current_motion_task) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotionState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotionState_

// alias to use template instance with default allocator
using MotionState =
  interface_protocol::msg::MotionState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__MOTION_STATE__STRUCT_HPP_
