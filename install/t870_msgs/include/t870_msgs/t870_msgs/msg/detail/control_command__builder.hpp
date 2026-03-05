// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from t870_msgs:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef T870_MSGS__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_
#define T870_MSGS__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "t870_msgs/msg/detail/control_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace t870_msgs
{

namespace msg
{

namespace builder
{

class Init_ControlCommand_steering
{
public:
  explicit Init_ControlCommand_steering(::t870_msgs::msg::ControlCommand & msg)
  : msg_(msg)
  {}
  ::t870_msgs::msg::ControlCommand steering(::t870_msgs::msg::ControlCommand::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return std::move(msg_);
  }

private:
  ::t870_msgs::msg::ControlCommand msg_;
};

class Init_ControlCommand_speed
{
public:
  Init_ControlCommand_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlCommand_steering speed(::t870_msgs::msg::ControlCommand::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_ControlCommand_steering(msg_);
  }

private:
  ::t870_msgs::msg::ControlCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::t870_msgs::msg::ControlCommand>()
{
  return t870_msgs::msg::builder::Init_ControlCommand_speed();
}

}  // namespace t870_msgs

#endif  // T870_MSGS__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_
