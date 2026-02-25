// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ev_msgs:msg/Cone.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__CONE__TRAITS_HPP_
#define EV_MSGS__MSG__DETAIL__CONE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ev_msgs/msg/detail/cone__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace ev_msgs
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

  // member: radius
  {
    out << "radius: ";
    rosidl_generator_traits::value_to_yaml(msg.radius, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
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

  // member: radius
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radius: ";
    rosidl_generator_traits::value_to_yaml(msg.radius, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
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

}  // namespace ev_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ev_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ev_msgs::msg::Cone & msg,
  std::ostream & out, size_t indentation = 0)
{
  ev_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ev_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ev_msgs::msg::Cone & msg)
{
  return ev_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ev_msgs::msg::Cone>()
{
  return "ev_msgs::msg::Cone";
}

template<>
inline const char * name<ev_msgs::msg::Cone>()
{
  return "ev_msgs/msg/Cone";
}

template<>
struct has_fixed_size<ev_msgs::msg::Cone>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<ev_msgs::msg::Cone>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<ev_msgs::msg::Cone>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EV_MSGS__MSG__DETAIL__CONE__TRAITS_HPP_
