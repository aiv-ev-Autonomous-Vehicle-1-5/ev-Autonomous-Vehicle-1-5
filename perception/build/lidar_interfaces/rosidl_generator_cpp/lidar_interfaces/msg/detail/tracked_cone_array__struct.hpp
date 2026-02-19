// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_interfaces:msg/TrackedConeArray.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE_ARRAY__STRUCT_HPP_
#define LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE_ARRAY__STRUCT_HPP_

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
// Member 'cones'
#include "lidar_interfaces/msg/detail/tracked_cone__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lidar_interfaces__msg__TrackedConeArray __attribute__((deprecated))
#else
# define DEPRECATED__lidar_interfaces__msg__TrackedConeArray __declspec(deprecated)
#endif

namespace lidar_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrackedConeArray_
{
  using Type = TrackedConeArray_<ContainerAllocator>;

  explicit TrackedConeArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit TrackedConeArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _cones_type =
    std::vector<lidar_interfaces::msg::TrackedCone_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<lidar_interfaces::msg::TrackedCone_<ContainerAllocator>>>;
  _cones_type cones;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__cones(
    const std::vector<lidar_interfaces::msg::TrackedCone_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<lidar_interfaces::msg::TrackedCone_<ContainerAllocator>>> & _arg)
  {
    this->cones = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_interfaces__msg__TrackedConeArray
    std::shared_ptr<lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_interfaces__msg__TrackedConeArray
    std::shared_ptr<lidar_interfaces::msg::TrackedConeArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackedConeArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->cones != other.cones) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackedConeArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackedConeArray_

// alias to use template instance with default allocator
using TrackedConeArray =
  lidar_interfaces::msg::TrackedConeArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidar_interfaces

#endif  // LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE_ARRAY__STRUCT_HPP_
