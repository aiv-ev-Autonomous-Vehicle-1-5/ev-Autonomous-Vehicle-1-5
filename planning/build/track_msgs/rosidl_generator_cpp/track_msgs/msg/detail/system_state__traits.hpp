// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from track_msgs:msg/SystemState.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__TRAITS_HPP_
#define TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "track_msgs/msg/detail/system_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace track_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SystemState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SystemState & msg,
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SystemState & msg, bool use_flow_style = false)
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
  const track_msgs::msg::SystemState & msg,
  std::ostream & out, size_t indentation = 0)
{
  track_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use track_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const track_msgs::msg::SystemState & msg)
{
  return track_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<track_msgs::msg::SystemState>()
{
  return "track_msgs::msg::SystemState";
}

template<>
inline const char * name<track_msgs::msg::SystemState>()
{
  return "track_msgs/msg/SystemState";
}

template<>
struct has_fixed_size<track_msgs::msg::SystemState>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<track_msgs::msg::SystemState>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<track_msgs::msg::SystemState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TRACK_MSGS__MSG__DETAIL__SYSTEM_STATE__TRAITS_HPP_
