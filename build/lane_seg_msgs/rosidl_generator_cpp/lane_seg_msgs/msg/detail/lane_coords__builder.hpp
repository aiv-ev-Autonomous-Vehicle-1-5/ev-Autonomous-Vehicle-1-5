// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lane_seg_msgs:msg/LaneCoords.idl
// generated code does not contain a copyright notice

#ifndef LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__BUILDER_HPP_
#define LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lane_seg_msgs/msg/detail/lane_coords__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lane_seg_msgs
{

namespace msg
{

namespace builder
{

class Init_LaneCoords_line_y
{
public:
  explicit Init_LaneCoords_line_y(::lane_seg_msgs::msg::LaneCoords & msg)
  : msg_(msg)
  {}
  ::lane_seg_msgs::msg::LaneCoords line_y(::lane_seg_msgs::msg::LaneCoords::_line_y_type arg)
  {
    msg_.line_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lane_seg_msgs::msg::LaneCoords msg_;
};

class Init_LaneCoords_line_x
{
public:
  Init_LaneCoords_line_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LaneCoords_line_y line_x(::lane_seg_msgs::msg::LaneCoords::_line_x_type arg)
  {
    msg_.line_x = std::move(arg);
    return Init_LaneCoords_line_y(msg_);
  }

private:
  ::lane_seg_msgs::msg::LaneCoords msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lane_seg_msgs::msg::LaneCoords>()
{
  return lane_seg_msgs::msg::builder::Init_LaneCoords_line_x();
}

}  // namespace lane_seg_msgs

#endif  // LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__BUILDER_HPP_
