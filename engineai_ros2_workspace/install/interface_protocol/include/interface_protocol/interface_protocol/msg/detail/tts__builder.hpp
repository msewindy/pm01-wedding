// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface_protocol:msg/Tts.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE_PROTOCOL__MSG__DETAIL__TTS__BUILDER_HPP_
#define INTERFACE_PROTOCOL__MSG__DETAIL__TTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface_protocol/msg/detail/tts__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface_protocol
{

namespace msg
{

namespace builder
{

class Init_Tts_rate
{
public:
  explicit Init_Tts_rate(::interface_protocol::msg::Tts & msg)
  : msg_(msg)
  {}
  ::interface_protocol::msg::Tts rate(::interface_protocol::msg::Tts::_rate_type arg)
  {
    msg_.rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface_protocol::msg::Tts msg_;
};

class Init_Tts_speaker
{
public:
  explicit Init_Tts_speaker(::interface_protocol::msg::Tts & msg)
  : msg_(msg)
  {}
  Init_Tts_rate speaker(::interface_protocol::msg::Tts::_speaker_type arg)
  {
    msg_.speaker = std::move(arg);
    return Init_Tts_rate(msg_);
  }

private:
  ::interface_protocol::msg::Tts msg_;
};

class Init_Tts_language
{
public:
  explicit Init_Tts_language(::interface_protocol::msg::Tts & msg)
  : msg_(msg)
  {}
  Init_Tts_speaker language(::interface_protocol::msg::Tts::_language_type arg)
  {
    msg_.language = std::move(arg);
    return Init_Tts_speaker(msg_);
  }

private:
  ::interface_protocol::msg::Tts msg_;
};

class Init_Tts_text
{
public:
  Init_Tts_text()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Tts_language text(::interface_protocol::msg::Tts::_text_type arg)
  {
    msg_.text = std::move(arg);
    return Init_Tts_language(msg_);
  }

private:
  ::interface_protocol::msg::Tts msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface_protocol::msg::Tts>()
{
  return interface_protocol::msg::builder::Init_Tts_text();
}

}  // namespace interface_protocol

#endif  // INTERFACE_PROTOCOL__MSG__DETAIL__TTS__BUILDER_HPP_
