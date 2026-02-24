// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from track_msgs:msg/Cone.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__CONE__BUILDER_HPP_
#define TRACK_MSGS__MSG__DETAIL__CONE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "track_msgs/msg/detail/cone__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace track_msgs
{

namespace msg
{

namespace builder
{

class Init_Cone_label
{
public:
  explicit Init_Cone_label(::track_msgs::msg::Cone & msg)
  : msg_(msg)
  {}
  ::track_msgs::msg::Cone label(::track_msgs::msg::Cone::_label_type arg)
  {
    msg_.label = std::move(arg);
    return std::move(msg_);
  }

private:
  ::track_msgs::msg::Cone msg_;
};

class Init_Cone_confidence
{
public:
  explicit Init_Cone_confidence(::track_msgs::msg::Cone & msg)
  : msg_(msg)
  {}
  Init_Cone_label confidence(::track_msgs::msg::Cone::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Cone_label(msg_);
  }

private:
  ::track_msgs::msg::Cone msg_;
};

class Init_Cone_dimensions
{
public:
  explicit Init_Cone_dimensions(::track_msgs::msg::Cone & msg)
  : msg_(msg)
  {}
  Init_Cone_confidence dimensions(::track_msgs::msg::Cone::_dimensions_type arg)
  {
    msg_.dimensions = std::move(arg);
    return Init_Cone_confidence(msg_);
  }

private:
  ::track_msgs::msg::Cone msg_;
};

class Init_Cone_position
{
public:
  Init_Cone_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cone_dimensions position(::track_msgs::msg::Cone::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Cone_dimensions(msg_);
  }

private:
  ::track_msgs::msg::Cone msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::track_msgs::msg::Cone>()
{
  return track_msgs::msg::builder::Init_Cone_position();
}

}  // namespace track_msgs

#endif  // TRACK_MSGS__MSG__DETAIL__CONE__BUILDER_HPP_
