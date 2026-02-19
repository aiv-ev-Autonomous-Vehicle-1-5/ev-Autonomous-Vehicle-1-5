// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_interfaces:msg/Cone.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_INTERFACES__MSG__DETAIL__CONE__BUILDER_HPP_
#define LIDAR_INTERFACES__MSG__DETAIL__CONE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lidar_interfaces/msg/detail/cone__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lidar_interfaces
{

namespace msg
{

namespace builder
{

class Init_Cone_label
{
public:
  explicit Init_Cone_label(::lidar_interfaces::msg::Cone & msg)
  : msg_(msg)
  {}
  ::lidar_interfaces::msg::Cone label(::lidar_interfaces::msg::Cone::_label_type arg)
  {
    msg_.label = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_interfaces::msg::Cone msg_;
};

class Init_Cone_confidence
{
public:
  explicit Init_Cone_confidence(::lidar_interfaces::msg::Cone & msg)
  : msg_(msg)
  {}
  Init_Cone_label confidence(::lidar_interfaces::msg::Cone::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Cone_label(msg_);
  }

private:
  ::lidar_interfaces::msg::Cone msg_;
};

class Init_Cone_dimensions
{
public:
  explicit Init_Cone_dimensions(::lidar_interfaces::msg::Cone & msg)
  : msg_(msg)
  {}
  Init_Cone_confidence dimensions(::lidar_interfaces::msg::Cone::_dimensions_type arg)
  {
    msg_.dimensions = std::move(arg);
    return Init_Cone_confidence(msg_);
  }

private:
  ::lidar_interfaces::msg::Cone msg_;
};

class Init_Cone_position
{
public:
  explicit Init_Cone_position(::lidar_interfaces::msg::Cone & msg)
  : msg_(msg)
  {}
  Init_Cone_dimensions position(::lidar_interfaces::msg::Cone::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Cone_dimensions(msg_);
  }

private:
  ::lidar_interfaces::msg::Cone msg_;
};

class Init_Cone_header
{
public:
  Init_Cone_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cone_position header(::lidar_interfaces::msg::Cone::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Cone_position(msg_);
  }

private:
  ::lidar_interfaces::msg::Cone msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_interfaces::msg::Cone>()
{
  return lidar_interfaces::msg::builder::Init_Cone_header();
}

}  // namespace lidar_interfaces

#endif  // LIDAR_INTERFACES__MSG__DETAIL__CONE__BUILDER_HPP_
