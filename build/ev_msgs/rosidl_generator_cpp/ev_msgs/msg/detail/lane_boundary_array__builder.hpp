// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ev_msgs:msg/LaneBoundaryArray.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__BUILDER_HPP_
#define EV_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ev_msgs/msg/detail/lane_boundary_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ev_msgs
{

namespace msg
{

namespace builder
{

class Init_LaneBoundaryArray_boundaries
{
public:
  explicit Init_LaneBoundaryArray_boundaries(::ev_msgs::msg::LaneBoundaryArray & msg)
  : msg_(msg)
  {}
  ::ev_msgs::msg::LaneBoundaryArray boundaries(::ev_msgs::msg::LaneBoundaryArray::_boundaries_type arg)
  {
    msg_.boundaries = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev_msgs::msg::LaneBoundaryArray msg_;
};

class Init_LaneBoundaryArray_header
{
public:
  Init_LaneBoundaryArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LaneBoundaryArray_boundaries header(::ev_msgs::msg::LaneBoundaryArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LaneBoundaryArray_boundaries(msg_);
  }

private:
  ::ev_msgs::msg::LaneBoundaryArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev_msgs::msg::LaneBoundaryArray>()
{
  return ev_msgs::msg::builder::Init_LaneBoundaryArray_header();
}

}  // namespace ev_msgs

#endif  // EV_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__BUILDER_HPP_
