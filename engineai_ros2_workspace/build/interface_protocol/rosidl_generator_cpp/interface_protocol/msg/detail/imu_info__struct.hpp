// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface_protocol:msg/ImuInfo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__STRUCT_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__STRUCT_HPP_

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
// Member 'quaternion'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
// Member 'rpy'
// Member 'linear_acceleration'
// Member 'angular_velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__interface_protocol__msg__ImuInfo __attribute__((deprecated))
#else
# define DEPRECATED__interface_protocol__msg__ImuInfo __declspec(deprecated)
#endif

namespace interface_protocol
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ImuInfo_
{
  using Type = ImuInfo_<ContainerAllocator>;

  explicit ImuInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    quaternion(_init),
    rpy(_init),
    linear_acceleration(_init),
    angular_velocity(_init)
  {
    (void)_init;
  }

  explicit ImuInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    quaternion(_alloc, _init),
    rpy(_alloc, _init),
    linear_acceleration(_alloc, _init),
    angular_velocity(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _quaternion_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _quaternion_type quaternion;
  using _rpy_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _rpy_type rpy;
  using _linear_acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _linear_acceleration_type linear_acceleration;
  using _angular_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _angular_velocity_type angular_velocity;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__quaternion(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->quaternion = _arg;
    return *this;
  }
  Type & set__rpy(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->rpy = _arg;
    return *this;
  }
  Type & set__linear_acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->linear_acceleration = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_protocol::msg::ImuInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_protocol::msg::ImuInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_protocol::msg::ImuInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_protocol::msg::ImuInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_protocol::msg::ImuInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::msg::ImuInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_protocol::msg::ImuInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::msg::ImuInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_protocol::msg::ImuInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_protocol::msg::ImuInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_protocol__msg__ImuInfo
    std::shared_ptr<interface_protocol::msg::ImuInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_protocol__msg__ImuInfo
    std::shared_ptr<interface_protocol::msg::ImuInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImuInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->quaternion != other.quaternion) {
      return false;
    }
    if (this->rpy != other.rpy) {
      return false;
    }
    if (this->linear_acceleration != other.linear_acceleration) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImuInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImuInfo_

// alias to use template instance with default allocator
using ImuInfo =
  interface_protocol::msg::ImuInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__IMU_INFO__STRUCT_HPP_
