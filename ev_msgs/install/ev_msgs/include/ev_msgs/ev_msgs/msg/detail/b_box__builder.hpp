// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ev_msgs:msg/BBox.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__B_BOX__BUILDER_HPP_
#define EV_MSGS__MSG__DETAIL__B_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ev_msgs/msg/detail/b_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ev_msgs
{

namespace msg
{

namespace builder
{

class Init_BBox_label
{
public:
  explicit Init_BBox_label(::ev_msgs::msg::BBox & msg)
  : msg_(msg)
  {}
  ::ev_msgs::msg::BBox label(::ev_msgs::msg::BBox::_label_type arg)
  {
    msg_.label = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev_msgs::msg::BBox msg_;
};

class Init_BBox_confidence
{
public:
  explicit Init_BBox_confidence(::ev_msgs::msg::BBox & msg)
  : msg_(msg)
  {}
  Init_BBox_label confidence(::ev_msgs::msg::BBox::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_BBox_label(msg_);
  }

private:
  ::ev_msgs::msg::BBox msg_;
};

class Init_BBox_size_z
{
public:
  explicit Init_BBox_size_z(::ev_msgs::msg::BBox & msg)
  : msg_(msg)
  {}
  Init_BBox_confidence size_z(::ev_msgs::msg::BBox::_size_z_type arg)
  {
    msg_.size_z = std::move(arg);
    return Init_BBox_confidence(msg_);
  }

private:
  ::ev_msgs::msg::BBox msg_;
};

class Init_BBox_size_y
{
public:
  explicit Init_BBox_size_y(::ev_msgs::msg::BBox & msg)
  : msg_(msg)
  {}
  Init_BBox_size_z size_y(::ev_msgs::msg::BBox::_size_y_type arg)
  {
    msg_.size_y = std::move(arg);
    return Init_BBox_size_z(msg_);
  }

private:
  ::ev_msgs::msg::BBox msg_;
};

class Init_BBox_size_x
{
public:
  explicit Init_BBox_size_x(::ev_msgs::msg::BBox & msg)
  : msg_(msg)
  {}
  Init_BBox_size_y size_x(::ev_msgs::msg::BBox::_size_x_type arg)
  {
    msg_.size_x = std::move(arg);
    return Init_BBox_size_y(msg_);
  }

private:
  ::ev_msgs::msg::BBox msg_;
};

class Init_BBox_position
{
public:
  Init_BBox_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BBox_size_x position(::ev_msgs::msg::BBox::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_BBox_size_x(msg_);
  }

private:
  ::ev_msgs::msg::BBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev_msgs::msg::BBox>()
{
  return ev_msgs::msg::builder::Init_BBox_position();
}

}  // namespace ev_msgs

#endif  // EV_MSGS__MSG__DETAIL__B_BOX__BUILDER_HPP_
