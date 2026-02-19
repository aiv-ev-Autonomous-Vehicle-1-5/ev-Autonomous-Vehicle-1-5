// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_interfaces:msg/TrackedCone.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE__BUILDER_HPP_
#define LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lidar_interfaces/msg/detail/tracked_cone__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lidar_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrackedCone_label
{
public:
  explicit Init_TrackedCone_label(::lidar_interfaces::msg::TrackedCone & msg)
  : msg_(msg)
  {}
  ::lidar_interfaces::msg::TrackedCone label(::lidar_interfaces::msg::TrackedCone::_label_type arg)
  {
    msg_.label = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_interfaces::msg::TrackedCone msg_;
};

class Init_TrackedCone_confidence
{
public:
  explicit Init_TrackedCone_confidence(::lidar_interfaces::msg::TrackedCone & msg)
  : msg_(msg)
  {}
  Init_TrackedCone_label confidence(::lidar_interfaces::msg::TrackedCone::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_TrackedCone_label(msg_);
  }

private:
  ::lidar_interfaces::msg::TrackedCone msg_;
};

class Init_TrackedCone_dimensions
{
public:
  explicit Init_TrackedCone_dimensions(::lidar_interfaces::msg::TrackedCone & msg)
  : msg_(msg)
  {}
  Init_TrackedCone_confidence dimensions(::lidar_interfaces::msg::TrackedCone::_dimensions_type arg)
  {
    msg_.dimensions = std::move(arg);
    return Init_TrackedCone_confidence(msg_);
  }

private:
  ::lidar_interfaces::msg::TrackedCone msg_;
};

class Init_TrackedCone_velocity
{
public:
  explicit Init_TrackedCone_velocity(::lidar_interfaces::msg::TrackedCone & msg)
  : msg_(msg)
  {}
  Init_TrackedCone_dimensions velocity(::lidar_interfaces::msg::TrackedCone::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_TrackedCone_dimensions(msg_);
  }

private:
  ::lidar_interfaces::msg::TrackedCone msg_;
};

class Init_TrackedCone_position
{
public:
  explicit Init_TrackedCone_position(::lidar_interfaces::msg::TrackedCone & msg)
  : msg_(msg)
  {}
  Init_TrackedCone_velocity position(::lidar_interfaces::msg::TrackedCone::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_TrackedCone_velocity(msg_);
  }

private:
  ::lidar_interfaces::msg::TrackedCone msg_;
};

class Init_TrackedCone_track_id
{
public:
  explicit Init_TrackedCone_track_id(::lidar_interfaces::msg::TrackedCone & msg)
  : msg_(msg)
  {}
  Init_TrackedCone_position track_id(::lidar_interfaces::msg::TrackedCone::_track_id_type arg)
  {
    msg_.track_id = std::move(arg);
    return Init_TrackedCone_position(msg_);
  }

private:
  ::lidar_interfaces::msg::TrackedCone msg_;
};

class Init_TrackedCone_header
{
public:
  Init_TrackedCone_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackedCone_track_id header(::lidar_interfaces::msg::TrackedCone::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrackedCone_track_id(msg_);
  }

private:
  ::lidar_interfaces::msg::TrackedCone msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_interfaces::msg::TrackedCone>()
{
  return lidar_interfaces::msg::builder::Init_TrackedCone_header();
}

}  // namespace lidar_interfaces

#endif  // LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE__BUILDER_HPP_
