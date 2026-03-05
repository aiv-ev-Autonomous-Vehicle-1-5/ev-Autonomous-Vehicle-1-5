// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from t870_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef T870_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_
#define T870_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "t870_msgs/msg/detail/feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace t870_msgs
{

namespace msg
{

namespace builder
{

class Init_Feedback_heartbeat
{
public:
  explicit Init_Feedback_heartbeat(::t870_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  ::t870_msgs::msg::Feedback heartbeat(::t870_msgs::msg::Feedback::_heartbeat_type arg)
  {
    msg_.heartbeat = std::move(arg);
    return std::move(msg_);
  }

private:
  ::t870_msgs::msg::Feedback msg_;
};

class Init_Feedback_steering
{
public:
  explicit Init_Feedback_steering(::t870_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_heartbeat steering(::t870_msgs::msg::Feedback::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return Init_Feedback_heartbeat(msg_);
  }

private:
  ::t870_msgs::msg::Feedback msg_;
};

class Init_Feedback_speed
{
public:
  explicit Init_Feedback_speed(::t870_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_steering speed(::t870_msgs::msg::Feedback::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_Feedback_steering(msg_);
  }

private:
  ::t870_msgs::msg::Feedback msg_;
};

class Init_Feedback_gear
{
public:
  explicit Init_Feedback_gear(::t870_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_speed gear(::t870_msgs::msg::Feedback::_gear_type arg)
  {
    msg_.gear = std::move(arg);
    return Init_Feedback_speed(msg_);
  }

private:
  ::t870_msgs::msg::Feedback msg_;
};

class Init_Feedback_emergency_stop
{
public:
  explicit Init_Feedback_emergency_stop(::t870_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_gear emergency_stop(::t870_msgs::msg::Feedback::_emergency_stop_type arg)
  {
    msg_.emergency_stop = std::move(arg);
    return Init_Feedback_gear(msg_);
  }

private:
  ::t870_msgs::msg::Feedback msg_;
};

class Init_Feedback_manual_mode
{
public:
  explicit Init_Feedback_manual_mode(::t870_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_emergency_stop manual_mode(::t870_msgs::msg::Feedback::_manual_mode_type arg)
  {
    msg_.manual_mode = std::move(arg);
    return Init_Feedback_emergency_stop(msg_);
  }

private:
  ::t870_msgs::msg::Feedback msg_;
};

class Init_Feedback_header
{
public:
  Init_Feedback_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Feedback_manual_mode header(::t870_msgs::msg::Feedback::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Feedback_manual_mode(msg_);
  }

private:
  ::t870_msgs::msg::Feedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::t870_msgs::msg::Feedback>()
{
  return t870_msgs::msg::builder::Init_Feedback_header();
}

}  // namespace t870_msgs

#endif  // T870_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_
