// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lane_seg_msgs:msg/LaneCoords.idl
// generated code does not contain a copyright notice

#ifndef LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__TRAITS_HPP_
#define LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lane_seg_msgs/msg/detail/lane_coords__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace lane_seg_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LaneCoords & msg,
  std::ostream & out)
{
  out << "{";
  // member: line_x
  {
    if (msg.line_x.size() == 0) {
      out << "line_x: []";
    } else {
      out << "line_x: [";
      size_t pending_items = msg.line_x.size();
      for (auto item : msg.line_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: line_y
  {
    if (msg.line_y.size() == 0) {
      out << "line_y: []";
    } else {
      out << "line_y: [";
      size_t pending_items = msg.line_y.size();
      for (auto item : msg.line_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LaneCoords & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: line_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.line_x.size() == 0) {
      out << "line_x: []\n";
    } else {
      out << "line_x:\n";
      for (auto item : msg.line_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: line_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.line_y.size() == 0) {
      out << "line_y: []\n";
    } else {
      out << "line_y:\n";
      for (auto item : msg.line_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LaneCoords & msg, bool use_flow_style = false)
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

}  // namespace lane_seg_msgs

namespace rosidl_generator_traits
{

[[deprecated("use lane_seg_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const lane_seg_msgs::msg::LaneCoords & msg,
  std::ostream & out, size_t indentation = 0)
{
  lane_seg_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lane_seg_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const lane_seg_msgs::msg::LaneCoords & msg)
{
  return lane_seg_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<lane_seg_msgs::msg::LaneCoords>()
{
  return "lane_seg_msgs::msg::LaneCoords";
}

template<>
inline const char * name<lane_seg_msgs::msg::LaneCoords>()
{
  return "lane_seg_msgs/msg/LaneCoords";
}

template<>
struct has_fixed_size<lane_seg_msgs::msg::LaneCoords>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lane_seg_msgs::msg::LaneCoords>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lane_seg_msgs::msg::LaneCoords>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__TRAITS_HPP_
