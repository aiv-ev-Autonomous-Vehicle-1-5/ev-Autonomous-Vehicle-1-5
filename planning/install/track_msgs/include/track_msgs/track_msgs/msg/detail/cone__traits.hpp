// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from track_msgs:msg/Cone.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__CONE__TRAITS_HPP_
#define TRACK_MSGS__MSG__DETAIL__CONE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "track_msgs/msg/detail/cone__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'dimensions'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace track_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Cone & msg,
  std::ostream & out)
{
  out << "{";
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

}  // namespace track_msgs

namespace rosidl_generator_traits
{

[[deprecated("use track_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const track_msgs::msg::Cone & msg,
  std::ostream & out, size_t indentation = 0)
{
  track_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use track_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const track_msgs::msg::Cone & msg)
{
  return track_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<track_msgs::msg::Cone>()
{
  return "track_msgs::msg::Cone";
}

template<>
inline const char * name<track_msgs::msg::Cone>()
{
  return "track_msgs/msg/Cone";
}

template<>
struct has_fixed_size<track_msgs::msg::Cone>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<track_msgs::msg::Cone>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<track_msgs::msg::Cone>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRACK_MSGS__MSG__DETAIL__CONE__TRAITS_HPP_
