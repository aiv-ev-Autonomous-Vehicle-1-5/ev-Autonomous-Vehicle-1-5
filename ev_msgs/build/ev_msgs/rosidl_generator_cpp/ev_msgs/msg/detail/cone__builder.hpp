// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ev_msgs:msg/Cone.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__CONE__BUILDER_HPP_
#define EV_MSGS__MSG__DETAIL__CONE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ev_msgs/msg/detail/cone__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ev_msgs
{

namespace msg
{

namespace builder
{

class Init_Cone_label
{
public:
  explicit Init_Cone_label(::ev_msgs::msg::Cone & msg)
  : msg_(msg)
  {}
  ::ev_msgs::msg::Cone label(::ev_msgs::msg::Cone::_label_type arg)
  {
    msg_.label = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev_msgs::msg::Cone msg_;
};

class Init_Cone_confidence
{
public:
  explicit Init_Cone_confidence(::ev_msgs::msg::Cone & msg)
  : msg_(msg)
  {}
  Init_Cone_label confidence(::ev_msgs::msg::Cone::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Cone_label(msg_);
  }

private:
  ::ev_msgs::msg::Cone msg_;
};

class Init_Cone_height
{
public:
  explicit Init_Cone_height(::ev_msgs::msg::Cone & msg)
  : msg_(msg)
  {}
  Init_Cone_confidence height(::ev_msgs::msg::Cone::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_Cone_confidence(msg_);
  }

private:
  ::ev_msgs::msg::Cone msg_;
};

class Init_Cone_radius
{
public:
  explicit Init_Cone_radius(::ev_msgs::msg::Cone & msg)
  : msg_(msg)
  {}
  Init_Cone_height radius(::ev_msgs::msg::Cone::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return Init_Cone_height(msg_);
  }

private:
  ::ev_msgs::msg::Cone msg_;
};

class Init_Cone_position
{
public:
  Init_Cone_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cone_radius position(::ev_msgs::msg::Cone::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Cone_radius(msg_);
  }

private:
  ::ev_msgs::msg::Cone msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev_msgs::msg::Cone>()
{
  return ev_msgs::msg::builder::Init_Cone_position();
}

}  // namespace ev_msgs

#endif  // EV_MSGS__MSG__DETAIL__CONE__BUILDER_HPP_
