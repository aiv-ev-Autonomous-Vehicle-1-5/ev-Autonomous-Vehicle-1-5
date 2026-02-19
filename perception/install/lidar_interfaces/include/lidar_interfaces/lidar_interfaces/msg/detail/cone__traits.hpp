// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_interfaces:msg/Cone.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_INTERFACES__MSG__DETAIL__CONE__TRAITS_HPP_
#define LIDAR_INTERFACES__MSG__DETAIL__CONE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lidar_interfaces/msg/detail/cone__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'dimensions'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace lidar_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Cone & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: dimensions
  {
    out << "dimensions: ";
    to_flow_style_yaml(msg.dimensions, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: label
  {
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Cone & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: dimensions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dimensions:\n";
    to_block_style_yaml(msg.dimensions, out, indentation + 2);
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Cone & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace lidar_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use lidar_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const lidar_interfaces::msg::Cone & msg,
  std::ostream & out, size_t indentation = 0)
{
  lidar_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lidar_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const lidar_interfaces::msg::Cone & msg)
{
  return lidar_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<lidar_interfaces::msg::Cone>()
{
  return "lidar_interfaces::msg::Cone";
}

template<>
inline const char * name<lidar_interfaces::msg::Cone>()
{
  return "lidar_interfaces/msg/Cone";
}

template<>
struct has_fixed_size<lidar_interfaces::msg::Cone>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<lidar_interfaces::msg::Cone>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<lidar_interfaces::msg::Cone>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_INTERFACES__MSG__DETAIL__CONE__TRAITS_HPP_
