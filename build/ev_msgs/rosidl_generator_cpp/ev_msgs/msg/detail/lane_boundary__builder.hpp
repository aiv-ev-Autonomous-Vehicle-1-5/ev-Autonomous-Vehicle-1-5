// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ev_msgs:msg/LaneBoundary.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__LANE_BOUNDARY__BUILDER_HPP_
#define EV_MSGS__MSG__DETAIL__LANE_BOUNDARY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ev_msgs/msg/detail/lane_boundary__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ev_msgs
{

namespace msg
{

namespace builder
{

class Init_LaneBoundary_confidence
{
public:
  explicit Init_LaneBoundary_confidence(::ev_msgs::msg::LaneBoundary & msg)
  : msg_(msg)
  {}
  ::ev_msgs::msg::LaneBoundary confidence(::ev_msgs::msg::LaneBoundary::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev_msgs::msg::LaneBoundary msg_;
};

class Init_LaneBoundary_points
{
public:
  explicit Init_LaneBoundary_points(::ev_msgs::msg::LaneBoundary & msg)
  : msg_(msg)
  {}
  Init_LaneBoundary_confidence points(::ev_msgs::msg::LaneBoundary::_points_type arg)
  {
    msg_.points = std::move(arg);
    return Init_LaneBoundary_confidence(msg_);
  }

private:
  ::ev_msgs::msg::LaneBoundary msg_;
};

class Init_LaneBoundary_header
{
public:
  Init_LaneBoundary_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LaneBoundary_points header(::ev_msgs::msg::LaneBoundary::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LaneBoundary_points(msg_);
  }

private:
  ::ev_msgs::msg::LaneBoundary msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev_msgs::msg::LaneBoundary>()
{
  return ev_msgs::msg::builder::Init_LaneBoundary_header();
}

}  // namespace ev_msgs

#endif  // EV_MSGS__MSG__DETAIL__LANE_BOUNDARY__BUILDER_HPP_
