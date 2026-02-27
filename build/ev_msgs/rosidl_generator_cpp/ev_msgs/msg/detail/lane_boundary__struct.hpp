// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ev_msgs:msg/LaneBoundary.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_HPP_
#define EV_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_HPP_

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
# define DEPRECATED__ev_msgs__msg__LaneBoundary __attribute__((deprecated))
#else
# define DEPRECATED__ev_msgs__msg__LaneBoundary __declspec(deprecated)
#endif

namespace ev_msgs
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
      this->confidence = 0.0f;
    }
  }

  explicit LaneBoundary_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
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
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ev_msgs::msg::LaneBoundary_<ContainerAllocator> *;
  using ConstRawPtr =
    const ev_msgs::msg::LaneBoundary_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ev_msgs::msg::LaneBoundary_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ev_msgs::msg::LaneBoundary_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ev_msgs::msg::LaneBoundary_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ev_msgs::msg::LaneBoundary_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ev_msgs::msg::LaneBoundary_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ev_msgs::msg::LaneBoundary_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ev_msgs::msg::LaneBoundary_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ev_msgs::msg::LaneBoundary_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ev_msgs__msg__LaneBoundary
    std::shared_ptr<ev_msgs::msg::LaneBoundary_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ev_msgs__msg__LaneBoundary
    std::shared_ptr<ev_msgs::msg::LaneBoundary_<ContainerAllocator> const>
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
  ev_msgs::msg::LaneBoundary_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ev_msgs

#endif  // EV_MSGS__MSG__DETAIL__LANE_BOUNDARY__STRUCT_HPP_
