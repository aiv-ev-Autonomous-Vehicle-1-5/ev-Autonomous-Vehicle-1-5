// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from track_msgs:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__BUILDER_HPP_
#define TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "track_msgs/msg/detail/system_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace track_msgs
{

namespace msg
{

namespace builder
{

class Init_SystemState_state
{
public:
  explicit Init_SystemState_state(::track_msgs::msg::SystemState & msg)
  : msg_(msg)
  {}
  ::track_msgs::msg::SystemState state(::track_msgs::msg::SystemState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::track_msgs::msg::SystemState msg_;
};

class Init_SystemState_header
{
public:
  Init_SystemState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SystemState_state header(::track_msgs::msg::SystemState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SystemState_state(msg_);
  }

private:
  ::track_msgs::msg::SystemState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::track_msgs::msg::SystemState>()
{
  return track_msgs::msg::builder::Init_SystemState_header();
}

}  // namespace track_msgs

#endif  // TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__BUILDER_HPP_
