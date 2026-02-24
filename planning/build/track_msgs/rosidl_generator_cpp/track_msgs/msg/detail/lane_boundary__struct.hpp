// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from track_msgs:msg/LaneBoundary.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_HPP_
#define TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__track_msgs__msg__LaneBoundary __attribute__((deprecated))
#else
# define DEPRECATED__track_msgs__msg__LaneBoundary __declspec(deprecated)
#endif

namespace track_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LaneBoundary_
{
  using Type = LaneBoundary_<ContainerAllocator>;

  explicit LaneBoundary_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->side = 0;
      this->confidence = 0.0f;
    }
  }

  explicit LaneBoundary_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->side = 0;
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _points_type points;
  using _side_type =
    uint8_t;
  _side_type side;
  using _confidence_type =
    float;
  _confidence_type confidence;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__side(
    const uint8_t & _arg)
  {
    this->side = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t LEFT =
    0u;
  static constexpr uint8_t RIGHT =
    1u;

  // pointer types
  using RawPtr =
    track_msgs::msg::LaneBoundary_<ContainerAllocator> *;
  using ConstRawPtr =
    const track_msgs::msg::LaneBoundary_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<track_msgs::msg::LaneBoundary_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<track_msgs::msg::LaneBoundary_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      track_msgs::msg::LaneBoundary_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<track_msgs::msg::LaneBoundary_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      track_msgs::msg::LaneBoundary_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<track_msgs::msg::LaneBoundary_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<track_msgs::msg::LaneBoundary_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<track_msgs::msg::LaneBoundary_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__track_msgs__msg__LaneBoundary
    std::shared_ptr<track_msgs::msg::LaneBoundary_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__track_msgs__msg__LaneBoundary
    std::shared_ptr<track_msgs::msg::LaneBoundary_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LaneBoundary_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->side != other.side) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const LaneBoundary_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LaneBoundary_

// alias to use template instance with default allocator
using LaneBoundary =
  track_msgs::msg::LaneBoundary_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LaneBoundary_<ContainerAllocator>::LEFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t LaneBoundary_<ContainerAllocator>::RIGHT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace track_msgs

#endif  // TRACK_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_HPP_
