// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from erp42_msgs:msg/ControlCommand.idl
// generated code does not contain a copyright notice

#ifndef ERP42_MSGS__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_
#define ERP42_MSGS__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "erp42_msgs/msg/detail/control_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace erp42_msgs
{

namespace msg
{

namespace builder
{

class Init_ControlCommand_brake
{
public:
  explicit Init_ControlCommand_brake(::erp42_msgs::msg::ControlCommand & msg)
  : msg_(msg)
  {}
  ::erp42_msgs::msg::ControlCommand brake(::erp42_msgs::msg::ControlCommand::_brake_type arg)
  {
    msg_.brake = std::move(arg);
    return std::move(msg_);
  }

private:
  ::erp42_msgs::msg::ControlCommand msg_;
};

class Init_ControlCommand_steering
{
public:
  explicit Init_ControlCommand_steering(::erp42_msgs::msg::ControlCommand & msg)
  : msg_(msg)
  {}
  Init_ControlCommand_brake steering(::erp42_msgs::msg::ControlCommand::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return Init_ControlCommand_brake(msg_);
  }

private:
  ::erp42_msgs::msg::ControlCommand msg_;
};

class Init_ControlCommand_speed
{
public:
  Init_ControlCommand_speed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlCommand_steering speed(::erp42_msgs::msg::ControlCommand::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_ControlCommand_steering(msg_);
  }

private:
  ::erp42_msgs::msg::ControlCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::erp42_msgs::msg::ControlCommand>()
{
  return erp42_msgs::msg::builder::Init_ControlCommand_speed();
}

}  // namespace erp42_msgs

#endif  // ERP42_MSGS__MSG__DETAIL__CONTROL_COMMAND__BUILDER_HPP_
