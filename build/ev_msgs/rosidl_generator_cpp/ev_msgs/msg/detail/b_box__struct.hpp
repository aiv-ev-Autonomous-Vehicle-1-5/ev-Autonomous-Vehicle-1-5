// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ev_msgs:msg/BBox.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__B_BOX__STRUCT_HPP_
#define EV_MSGS__MSG__DETAIL__B_BOX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ev_msgs__msg__BBox __attribute__((deprecated))
#else
# define DEPRECATED__ev_msgs__msg__BBox __declspec(deprecated)
#endif

namespace ev_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BBox_
{
  using Type = BBox_<ContainerAllocator>;

  explicit BBox_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->size_x = 0.0f;
      this->size_y = 0.0f;
      this->size_z = 0.0f;
      this->confidence = 0.0f;
      this->label = 0l;
    }
  }

  explicit BBox_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->size_x = 0.0f;
      this->size_y = 0.0f;
      this->size_z = 0.0f;
      this->confidence = 0.0f;
      this->label = 0l;
    }
  }

  // field types and members
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _size_x_type =
    float;
  _size_x_type size_x;
  using _size_y_type =
    float;
  _size_y_type size_y;
  using _size_z_type =
    float;
  _size_z_type size_z;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _label_type =
    int32_t;
  _label_type label;

  // setters for named parameter idiom
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__size_x(
    const float & _arg)
  {
    this->size_x = _arg;
    return *this;
  }
  Type & set__size_y(
    const float & _arg)
  {
    this->size_y = _arg;
    return *this;
  }
  Type & set__size_z(
    const float & _arg)
  {
    this->size_z = _arg;
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
    ev_msgs::msg::BBox_<ContainerAllocator> *;
  using ConstRawPtr =
    const ev_msgs::msg::BBox_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ev_msgs::msg::BBox_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ev_msgs::msg::BBox_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ev_msgs::msg::BBox_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ev_msgs::msg::BBox_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ev_msgs::msg::BBox_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ev_msgs::msg::BBox_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ev_msgs::msg::BBox_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ev_msgs::msg::BBox_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ev_msgs__msg__BBox
    std::shared_ptr<ev_msgs::msg::BBox_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ev_msgs__msg__BBox
    std::shared_ptr<ev_msgs::msg::BBox_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BBox_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->size_x != other.size_x) {
      return false;
    }
    if (this->size_y != other.size_y) {
      return false;
    }
    if (this->size_z != other.size_z) {
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
  bool operator!=(const BBox_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BBox_

// alias to use template instance with default allocator
using BBox =
  ev_msgs::msg::BBox_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ev_msgs

#endif  // EV_MSGS__MSG__DETAIL__B_BOX__STRUCT_HPP_
