// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ev_msgs:msg/BBoxArray.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__B_BOX_ARRAY__BUILDER_HPP_
#define EV_MSGS__MSG__DETAIL__B_BOX_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ev_msgs/msg/detail/b_box_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ev_msgs
{

namespace msg
{

namespace builder
{

class Init_BBoxArray_bboxes
{
public:
  explicit Init_BBoxArray_bboxes(::ev_msgs::msg::BBoxArray & msg)
  : msg_(msg)
  {}
  ::ev_msgs::msg::BBoxArray bboxes(::ev_msgs::msg::BBoxArray::_bboxes_type arg)
  {
    msg_.bboxes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev_msgs::msg::BBoxArray msg_;
};

class Init_BBoxArray_header
{
public:
  Init_BBoxArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BBoxArray_bboxes header(::ev_msgs::msg::BBoxArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BBoxArray_bboxes(msg_);
  }

private:
  ::ev_msgs::msg::BBoxArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev_msgs::msg::BBoxArray>()
{
  return ev_msgs::msg::builder::Init_BBoxArray_header();
}

}  // namespace ev_msgs

#endif  // EV_MSGS__MSG__DETAIL__B_BOX_ARRAY__BUILDER_HPP_
