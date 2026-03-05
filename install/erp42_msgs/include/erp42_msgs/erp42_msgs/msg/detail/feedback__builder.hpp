// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from erp42_msgs:msg/Feedback.idl
// generated code does not contain a copyright notice

#ifndef ERP42_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_
#define ERP42_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "erp42_msgs/msg/detail/feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace erp42_msgs
{

namespace msg
{

namespace builder
{

class Init_Feedback_heartbeat
{
public:
  explicit Init_Feedback_heartbeat(::erp42_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  ::erp42_msgs::msg::Feedback heartbeat(::erp42_msgs::msg::Feedback::_heartbeat_type arg)
  {
    msg_.heartbeat = std::move(arg);
    return std::move(msg_);
  }

private:
  ::erp42_msgs::msg::Feedback msg_;
};

class Init_Feedback_encoder_count
{
public:
  explicit Init_Feedback_encoder_count(::erp42_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_heartbeat encoder_count(::erp42_msgs::msg::Feedback::_encoder_count_type arg)
  {
    msg_.encoder_count = std::move(arg);
    return Init_Feedback_heartbeat(msg_);
  }

private:
  ::erp42_msgs::msg::Feedback msg_;
};

class Init_Feedback_brake
{
public:
  explicit Init_Feedback_brake(::erp42_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_encoder_count brake(::erp42_msgs::msg::Feedback::_brake_type arg)
  {
    msg_.brake = std::move(arg);
    return Init_Feedback_encoder_count(msg_);
  }

private:
  ::erp42_msgs::msg::Feedback msg_;
};

class Init_Feedback_steering
{
public:
  explicit Init_Feedback_steering(::erp42_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_brake steering(::erp42_msgs::msg::Feedback::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return Init_Feedback_brake(msg_);
  }

private:
  ::erp42_msgs::msg::Feedback msg_;
};

class Init_Feedback_speed
{
public:
  explicit Init_Feedback_speed(::erp42_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_steering speed(::erp42_msgs::msg::Feedback::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_Feedback_steering(msg_);
  }

private:
  ::erp42_msgs::msg::Feedback msg_;
};

class Init_Feedback_gear
{
public:
  explicit Init_Feedback_gear(::erp42_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_speed gear(::erp42_msgs::msg::Feedback::_gear_type arg)
  {
    msg_.gear = std::move(arg);
    return Init_Feedback_speed(msg_);
  }

private:
  ::erp42_msgs::msg::Feedback msg_;
};

class Init_Feedback_emergency_stop
{
public:
  explicit Init_Feedback_emergency_stop(::erp42_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_gear emergency_stop(::erp42_msgs::msg::Feedback::_emergency_stop_type arg)
  {
    msg_.emergency_stop = std::move(arg);
    return Init_Feedback_gear(msg_);
  }

private:
  ::erp42_msgs::msg::Feedback msg_;
};

class Init_Feedback_manual_mode
{
public:
  explicit Init_Feedback_manual_mode(::erp42_msgs::msg::Feedback & msg)
  : msg_(msg)
  {}
  Init_Feedback_emergency_stop manual_mode(::erp42_msgs::msg::Feedback::_manual_mode_type arg)
  {
    msg_.manual_mode = std::move(arg);
    return Init_Feedback_emergency_stop(msg_);
  }

private:
  ::erp42_msgs::msg::Feedback msg_;
};

class Init_Feedback_header
{
public:
  Init_Feedback_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Feedback_manual_mode header(::erp42_msgs::msg::Feedback::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Feedback_manual_mode(msg_);
  }

private:
  ::erp42_msgs::msg::Feedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::erp42_msgs::msg::Feedback>()
{
  return erp42_msgs::msg::builder::Init_Feedback_header();
}

}  // namespace erp42_msgs

#endif  // ERP42_MSGS__MSG__DETAIL__FEEDBACK__BUILDER_HPP_
