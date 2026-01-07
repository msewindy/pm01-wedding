// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface_protocol:srv/EnableMotor.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__STRUCT_HPP_
#define INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interface_protocol__srv__EnableMotor_Request __attribute__((deprecated))
#else
# define DEPRECATED__interface_protocol__srv__EnableMotor_Request __declspec(deprecated)
#endif

namespace interface_protocol
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EnableMotor_Request_
{
  using Type = EnableMotor_Request_<ContainerAllocator>;

  explicit EnableMotor_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable = false;
    }
  }

  explicit EnableMotor_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable = false;
    }
  }

  // field types and members
  using _enable_type =
    bool;
  _enable_type enable;

  // setters for named parameter idiom
  Type & set__enable(
    const bool & _arg)
  {
    this->enable = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_protocol::srv::EnableMotor_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_protocol::srv::EnableMotor_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_protocol::srv::EnableMotor_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_protocol::srv::EnableMotor_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_protocol::srv::EnableMotor_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::srv::EnableMotor_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_protocol::srv::EnableMotor_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::srv::EnableMotor_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_protocol::srv::EnableMotor_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_protocol::srv::EnableMotor_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_protocol__srv__EnableMotor_Request
    std::shared_ptr<interface_protocol::srv::EnableMotor_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_protocol__srv__EnableMotor_Request
    std::shared_ptr<interface_protocol::srv::EnableMotor_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EnableMotor_Request_ & other) const
  {
    if (this->enable != other.enable) {
      return false;
    }
    return true;
  }
  bool operator!=(const EnableMotor_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EnableMotor_Request_

// alias to use template instance with default allocator
using EnableMotor_Request =
  interface_protocol::srv::EnableMotor_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface_protocol


#ifndef _WIN32
# define DEPRECATED__interface_protocol__srv__EnableMotor_Response __attribute__((deprecated))
#else
# define DEPRECATED__interface_protocol__srv__EnableMotor_Response __declspec(deprecated)
#endif

namespace interface_protocol
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EnableMotor_Response_
{
  using Type = EnableMotor_Response_<ContainerAllocator>;

  explicit EnableMotor_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit EnableMotor_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_protocol::srv::EnableMotor_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_protocol::srv::EnableMotor_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_protocol::srv::EnableMotor_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_protocol::srv::EnableMotor_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_protocol::srv::EnableMotor_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::srv::EnableMotor_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_protocol::srv::EnableMotor_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::srv::EnableMotor_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_protocol::srv::EnableMotor_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_protocol::srv::EnableMotor_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_protocol__srv__EnableMotor_Response
    std::shared_ptr<interface_protocol::srv::EnableMotor_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_protocol__srv__EnableMotor_Response
    std::shared_ptr<interface_protocol::srv::EnableMotor_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EnableMotor_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const EnableMotor_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EnableMotor_Response_

// alias to use template instance with default allocator
using EnableMotor_Response =
  interface_protocol::srv::EnableMotor_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interface_protocol

namespace interface_protocol
{

namespace srv
{

struct EnableMotor
{
  using Request = interface_protocol::srv::EnableMotor_Request;
  using Response = interface_protocol::srv::EnableMotor_Response;
};

}  // namespace srv

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__SRV__DETAIL__ENABLE_MOTOR__STRUCT_HPP_
