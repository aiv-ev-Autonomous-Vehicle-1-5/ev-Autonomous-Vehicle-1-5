// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from t870_msgs:srv/ModeCommand.idl
// generated code does not contain a copyright notice

#ifndef T870_MSGS__SRV__DETAIL__MODE_COMMAND__BUILDER_HPP_
#define T870_MSGS__SRV__DETAIL__MODE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "t870_msgs/srv/detail/mode_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace t870_msgs
{

namespace srv
{

namespace builder
{

class Init_ModeCommand_Request_gear
{
public:
  explicit Init_ModeCommand_Request_gear(::t870_msgs::srv::ModeCommand_Request & msg)
  : msg_(msg)
  {}
  ::t870_msgs::srv::ModeCommand_Request gear(::t870_msgs::srv::ModeCommand_Request::_gear_type arg)
  {
    msg_.gear = std::move(arg);
    return std::move(msg_);
  }

private:
  ::t870_msgs::srv::ModeCommand_Request msg_;
};

class Init_ModeCommand_Request_emergency_stop
{
public:
  explicit Init_ModeCommand_Request_emergency_stop(::t870_msgs::srv::ModeCommand_Request & msg)
  : msg_(msg)
  {}
  Init_ModeCommand_Request_gear emergency_stop(::t870_msgs::srv::ModeCommand_Request::_emergency_stop_type arg)
  {
    msg_.emergency_stop = std::move(arg);
    return Init_ModeCommand_Request_gear(msg_);
  }

private:
  ::t870_msgs::srv::ModeCommand_Request msg_;
};

class Init_ModeCommand_Request_manual_mode
{
public:
  Init_ModeCommand_Request_manual_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ModeCommand_Request_emergency_stop manual_mode(::t870_msgs::srv::ModeCommand_Request::_manual_mode_type arg)
  {
    msg_.manual_mode = std::move(arg);
    return Init_ModeCommand_Request_emergency_stop(msg_);
  }

private:
  ::t870_msgs::srv::ModeCommand_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::t870_msgs::srv::ModeCommand_Request>()
{
  return t870_msgs::srv::builder::Init_ModeCommand_Request_manual_mode();
}

}  // namespace t870_msgs


namespace t870_msgs
{

namespace srv
{

namespace builder
{

class Init_ModeCommand_Response_success
{
public:
  Init_ModeCommand_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::t870_msgs::srv::ModeCommand_Response success(::t870_msgs::srv::ModeCommand_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::t870_msgs::srv::ModeCommand_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::t870_msgs::srv::ModeCommand_Response>()
{
  return t870_msgs::srv::builder::Init_ModeCommand_Response_success();
}

}  // namespace t870_msgs

#endif  // T870_MSGS__SRV__DETAIL__MODE_COMMAND__BUILDER_HPP_
