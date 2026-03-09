// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lane_seg_msgs:msg/LaneCoords.idl
// generated code does not contain a copyright notice

#ifndef LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__STRUCT_HPP_
#define LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__lane_seg_msgs__msg__LaneCoords __attribute__((deprecated))
#else
# define DEPRECATED__lane_seg_msgs__msg__LaneCoords __declspec(deprecated)
#endif

namespace lane_seg_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LaneCoords_
{
  using Type = LaneCoords_<ContainerAllocator>;

  explicit LaneCoords_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit LaneCoords_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _line_x_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _line_x_type line_x;
  using _line_y_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _line_y_type line_y;

  // setters for named parameter idiom
  Type & set__line_x(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->line_x = _arg;
    return *this;
  }
  Type & set__line_y(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->line_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lane_seg_msgs::msg::LaneCoords_<ContainerAllocator> *;
  using ConstRawPtr =
    const lane_seg_msgs::msg::LaneCoords_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lane_seg_msgs::msg::LaneCoords_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lane_seg_msgs::msg::LaneCoords_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lane_seg_msgs::msg::LaneCoords_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lane_seg_msgs::msg::LaneCoords_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lane_seg_msgs::msg::LaneCoords_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lane_seg_msgs::msg::LaneCoords_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lane_seg_msgs::msg::LaneCoords_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lane_seg_msgs::msg::LaneCoords_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lane_seg_msgs__msg__LaneCoords
    std::shared_ptr<lane_seg_msgs::msg::LaneCoords_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lane_seg_msgs__msg__LaneCoords
    std::shared_ptr<lane_seg_msgs::msg::LaneCoords_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaneCoords_ & other) const
  {
    if (this->line_x != other.line_x) {
      return false;
    }
    if (this->line_y != other.line_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaneCoords_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaneCoords_

// alias to use template instance with default allocator
using LaneCoords =
  lane_seg_msgs::msg::LaneCoords_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lane_seg_msgs

#endif  // LANE_SEG_MSGS__MSG__DETAIL__LANE_COORDS__STRUCT_HPP_
