// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from track_msgs:msg/PlannerStatus.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__STRUCT_HPP_
#define TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__track_msgs__msg__PlannerStatus __attribute__((deprecated))
#else
# define DEPRECATED__track_msgs__msg__PlannerStatus __declspec(deprecated)
#endif

namespace track_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PlannerStatus_
{
  using Type = PlannerStatus_<ContainerAllocator>;

  explicit PlannerStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
      this->reason = "";
    }
  }

  explicit PlannerStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
      this->reason = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _status_type =
    uint8_t;
  _status_type status;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t OK =
    0u;
  static constexpr uint8_t STOP =
    1u;
  static constexpr uint8_t INFEASIBLE =
    2u;
  static constexpr uint8_t STALE =
    3u;

  // pointer types
  using RawPtr =
    track_msgs::msg::PlannerStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const track_msgs::msg::PlannerStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<track_msgs::msg::PlannerStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<track_msgs::msg::PlannerStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      track_msgs::msg::PlannerStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<track_msgs::msg::PlannerStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      track_msgs::msg::PlannerStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<track_msgs::msg::PlannerStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<track_msgs::msg::PlannerStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<track_msgs::msg::PlannerStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__track_msgs__msg__PlannerStatus
    std::shared_ptr<track_msgs::msg::PlannerStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__track_msgs__msg__PlannerStatus
    std::shared_ptr<track_msgs::msg::PlannerStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlannerStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlannerStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlannerStatus_

// alias to use template instance with default allocator
using PlannerStatus =
  track_msgs::msg::PlannerStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlannerStatus_<ContainerAllocator>::OK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlannerStatus_<ContainerAllocator>::STOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlannerStatus_<ContainerAllocator>::INFEASIBLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t PlannerStatus_<ContainerAllocator>::STALE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace track_msgs

#endif  // TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__STRUCT_HPP_
