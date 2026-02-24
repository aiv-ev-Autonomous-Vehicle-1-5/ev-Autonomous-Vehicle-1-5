// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from track_msgs:msg/LaneBoundaryArray.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__BUILDER_HPP_
#define TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "track_msgs/msg/detail/lane_boundary_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace track_msgs
{

namespace msg
{

namespace builder
{

class Init_LaneBoundaryArray_boundaries
{
public:
  explicit Init_LaneBoundaryArray_boundaries(::track_msgs::msg::LaneBoundaryArray & msg)
  : msg_(msg)
  {}
  ::track_msgs::msg::LaneBoundaryArray boundaries(::track_msgs::msg::LaneBoundaryArray::_boundaries_type arg)
  {
    msg_.boundaries = std::move(arg);
    return std::move(msg_);
  }

private:
  ::track_msgs::msg::LaneBoundaryArray msg_;
};

class Init_LaneBoundaryArray_header
{
public:
  Init_LaneBoundaryArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LaneBoundaryArray_boundaries header(::track_msgs::msg::LaneBoundaryArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LaneBoundaryArray_boundaries(msg_);
  }

private:
  ::track_msgs::msg::LaneBoundaryArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::track_msgs::msg::LaneBoundaryArray>()
{
  return track_msgs::msg::builder::Init_LaneBoundaryArray_header();
}

}  // namespace track_msgs

#endif  // TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY_ARRAY__BUILDER_HPP_
