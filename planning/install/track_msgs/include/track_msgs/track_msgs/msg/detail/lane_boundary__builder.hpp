// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from track_msgs:msg/LaneBoundary.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__BUILDER_HPP_
#define TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "track_msgs/msg/detail/lane_boundary__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace track_msgs
{

namespace msg
{

namespace builder
{

class Init_LaneBoundary_confidence
{
public:
  explicit Init_LaneBoundary_confidence(::track_msgs::msg::LaneBoundary & msg)
  : msg_(msg)
  {}
  ::track_msgs::msg::LaneBoundary confidence(::track_msgs::msg::LaneBoundary::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::track_msgs::msg::LaneBoundary msg_;
};

class Init_LaneBoundary_side
{
public:
  explicit Init_LaneBoundary_side(::track_msgs::msg::LaneBoundary & msg)
  : msg_(msg)
  {}
  Init_LaneBoundary_confidence side(::track_msgs::msg::LaneBoundary::_side_type arg)
  {
    msg_.side = std::move(arg);
    return Init_LaneBoundary_confidence(msg_);
  }

private:
  ::track_msgs::msg::LaneBoundary msg_;
};

class Init_LaneBoundary_points
{
public:
  explicit Init_LaneBoundary_points(::track_msgs::msg::LaneBoundary & msg)
  : msg_(msg)
  {}
  Init_LaneBoundary_side points(::track_msgs::msg::LaneBoundary::_points_type arg)
  {
    msg_.points = std::move(arg);
    return Init_LaneBoundary_side(msg_);
  }

private:
  ::track_msgs::msg::LaneBoundary msg_;
};

class Init_LaneBoundary_header
{
public:
  Init_LaneBoundary_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LaneBoundary_points header(::track_msgs::msg::LaneBoundary::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LaneBoundary_points(msg_);
  }

private:
  ::track_msgs::msg::LaneBoundary msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::track_msgs::msg::LaneBoundary>()
{
  return track_msgs::msg::builder::Init_LaneBoundary_header();
}

}  // namespace track_msgs

#endif  // TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__BUILDER_HPP_
