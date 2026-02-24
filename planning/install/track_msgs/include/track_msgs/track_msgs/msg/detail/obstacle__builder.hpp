// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from track_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__OBSTACLE__BUILDER_HPP_
#define TRACK_MSGS__MSG__DETAIL__OBSTACLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "track_msgs/msg/detail/obstacle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace track_msgs
{

namespace msg
{

namespace builder
{

class Init_Obstacle_type
{
public:
  explicit Init_Obstacle_type(::track_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  ::track_msgs::msg::Obstacle type(::track_msgs::msg::Obstacle::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::track_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_velocity
{
public:
  explicit Init_Obstacle_velocity(::track_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_type velocity(::track_msgs::msg::Obstacle::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Obstacle_type(msg_);
  }

private:
  ::track_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_dimensions
{
public:
  explicit Init_Obstacle_dimensions(::track_msgs::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_velocity dimensions(::track_msgs::msg::Obstacle::_dimensions_type arg)
  {
    msg_.dimensions = std::move(arg);
    return Init_Obstacle_velocity(msg_);
  }

private:
  ::track_msgs::msg::Obstacle msg_;
};

class Init_Obstacle_position
{
public:
  Init_Obstacle_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Obstacle_dimensions position(::track_msgs::msg::Obstacle::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Obstacle_dimensions(msg_);
  }

private:
  ::track_msgs::msg::Obstacle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::track_msgs::msg::Obstacle>()
{
  return track_msgs::msg::builder::Init_Obstacle_position();
}

}  // namespace track_msgs

#endif  // TRACK_MSGS__MSG__DETAIL__OBSTACLE__BUILDER_HPP_
