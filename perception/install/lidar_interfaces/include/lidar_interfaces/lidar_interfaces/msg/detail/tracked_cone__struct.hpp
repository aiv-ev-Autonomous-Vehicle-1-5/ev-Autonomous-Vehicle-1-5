// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_interfaces:msg/TrackedCone.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE__STRUCT_HPP_
#define LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE__STRUCT_HPP_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
// Member 'dimensions'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lidar_interfaces__msg__TrackedCone __attribute__((deprecated))
#else
# define DEPRECATED__lidar_interfaces__msg__TrackedCone __declspec(deprecated)
#endif

namespace lidar_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrackedCone_
{
  using Type = TrackedCone_<ContainerAllocator>;

  explicit TrackedCone_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    velocity(_init),
    dimensions(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->track_id = 0ul;
      this->confidence = 0.0f;
      this->label = 0l;
    }
  }

  explicit TrackedCone_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init),
    dimensions(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->track_id = 0ul;
      this->confidence = 0.0f;
      this->label = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _track_id_type =
    uint32_t;
  _track_id_type track_id;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _dimensions_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _dimensions_type dimensions;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _label_type =
    int32_t;
  _label_type label;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__track_id(
    const uint32_t & _arg)
  {
    this->track_id = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__dimensions(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->dimensions = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__label(
    const int32_t & _arg)
  {
    this->label = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_interfaces::msg::TrackedCone_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_interfaces::msg::TrackedCone_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_interfaces::msg::TrackedCone_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_interfaces::msg::TrackedCone_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_interfaces::msg::TrackedCone_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_interfaces::msg::TrackedCone_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_interfaces::msg::TrackedCone_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_interfaces::msg::TrackedCone_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_interfaces::msg::TrackedCone_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_interfaces::msg::TrackedCone_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_interfaces__msg__TrackedCone
    std::shared_ptr<lidar_interfaces::msg::TrackedCone_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_interfaces__msg__TrackedCone
    std::shared_ptr<lidar_interfaces::msg::TrackedCone_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackedCone_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->track_id != other.track_id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->dimensions != other.dimensions) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->label != other.label) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackedCone_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackedCone_

// alias to use template instance with default allocator
using TrackedCone =
  lidar_interfaces::msg::TrackedCone_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidar_interfaces

#endif  // LIDAR_INTERFACES__MSG__DETAIL__TRACKED_CONE__STRUCT_HPP_
