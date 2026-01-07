// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface_protocol:msg/Tts.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__TTS__STRUCT_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__TTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interface_protocol__msg__Tts __attribute__((deprecated))
#else
# define DEPRECATED__interface_protocol__msg__Tts __declspec(deprecated)
#endif

namespace interface_protocol
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Tts_
{
  using Type = Tts_<ContainerAllocator>;

  explicit Tts_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->text = "";
      this->language = "";
      this->speaker = "";
      this->rate = 0ll;
    }
  }

  explicit Tts_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : text(_alloc),
    language(_alloc),
    speaker(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->text = "";
      this->language = "";
      this->speaker = "";
      this->rate = 0ll;
    }
  }

  // field types and members
  using _text_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _text_type text;
  using _language_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _language_type language;
  using _speaker_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _speaker_type speaker;
  using _rate_type =
    int64_t;
  _rate_type rate;

  // setters for named parameter idiom
  Type & set__text(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->text = _arg;
    return *this;
  }
  Type & set__language(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->language = _arg;
    return *this;
  }
  Type & set__speaker(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->speaker = _arg;
    return *this;
  }
  Type & set__rate(
    const int64_t & _arg)
  {
    this->rate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface_protocol::msg::Tts_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface_protocol::msg::Tts_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface_protocol::msg::Tts_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface_protocol::msg::Tts_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface_protocol::msg::Tts_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::msg::Tts_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface_protocol::msg::Tts_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface_protocol::msg::Tts_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface_protocol::msg::Tts_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface_protocol::msg::Tts_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface_protocol__msg__Tts
    std::shared_ptr<interface_protocol::msg::Tts_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface_protocol__msg__Tts
    std::shared_ptr<interface_protocol::msg::Tts_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Tts_ & other) const
  {
    if (this->text != other.text) {
      return false;
    }
    if (this->language != other.language) {
      return false;
    }
    if (this->speaker != other.speaker) {
      return false;
    }
    if (this->rate != other.rate) {
      return false;
    }
    return true;
  }
  bool operator!=(const Tts_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Tts_

// alias to use template instance with default allocator
using Tts =
  interface_protocol::msg::Tts_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__TTS__STRUCT_HPP_
