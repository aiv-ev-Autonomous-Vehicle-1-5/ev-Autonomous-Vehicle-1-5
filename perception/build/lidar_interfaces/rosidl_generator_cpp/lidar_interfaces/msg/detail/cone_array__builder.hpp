// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_interfaces:msg/ConeArray.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_INTERFACES__MSG__DETAIL__CONE_ARRAY__BUILDER_HPP_
#define LIDAR_INTERFACES__MSG__DETAIL__CONE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lidar_interfaces/msg/detail/cone_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lidar_interfaces
{

namespace msg
{

namespace builder
{

class Init_ConeArray_cones
{
public:
  explicit Init_ConeArray_cones(::lidar_interfaces::msg::ConeArray & msg)
  : msg_(msg)
  {}
  ::lidar_interfaces::msg::ConeArray cones(::lidar_interfaces::msg::ConeArray::_cones_type arg)
  {
    msg_.cones = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_interfaces::msg::ConeArray msg_;
};

class Init_ConeArray_header
{
public:
  Init_ConeArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeArray_cones header(::lidar_interfaces::msg::ConeArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ConeArray_cones(msg_);
  }

private:
  ::lidar_interfaces::msg::ConeArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_interfaces::msg::ConeArray>()
{
  return lidar_interfaces::msg::builder::Init_ConeArray_header();
}

}  // namespace lidar_interfaces

#endif  // LIDAR_INTERFACES__MSG__DETAIL__CONE_ARRAY__BUILDER_HPP_
