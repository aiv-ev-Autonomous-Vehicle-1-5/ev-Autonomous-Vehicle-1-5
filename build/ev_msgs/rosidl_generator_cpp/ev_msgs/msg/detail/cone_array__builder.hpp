// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ev_msgs:msg/ConeArray.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__CONE_ARRAY__BUILDER_HPP_
#define EV_MSGS__MSG__DETAIL__CONE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ev_msgs/msg/detail/cone_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ev_msgs
{

namespace msg
{

namespace builder
{

class Init_ConeArray_cones
{
public:
  explicit Init_ConeArray_cones(::ev_msgs::msg::ConeArray & msg)
  : msg_(msg)
  {}
  ::ev_msgs::msg::ConeArray cones(::ev_msgs::msg::ConeArray::_cones_type arg)
  {
    msg_.cones = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev_msgs::msg::ConeArray msg_;
};

class Init_ConeArray_header
{
public:
  Init_ConeArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeArray_cones header(::ev_msgs::msg::ConeArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ConeArray_cones(msg_);
  }

private:
  ::ev_msgs::msg::ConeArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev_msgs::msg::ConeArray>()
{
  return ev_msgs::msg::builder::Init_ConeArray_header();
}

}  // namespace ev_msgs

#endif  // EV_MSGS__MSG__DETAIL__CONE_ARRAY__BUILDER_HPP_
