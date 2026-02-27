// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ev_msgs:msg/BBoxArray.idl
// generated code does not contain a copyright notice

#ifndef EV_MSGS__MSG__DETAIL__B_BOX_ARRAY__STRUCT_HPP_
#define EV_MSGS__MSG__DETAIL__B_BOX_ARRAY__STRUCT_HPP_

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
// Member 'bboxes'
#include "ev_msgs/msg/detail/b_box__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ev_msgs__msg__BBoxArray __attribute__((deprecated))
#else
# define DEPRECATED__ev_msgs__msg__BBoxArray __declspec(deprecated)
#endif

namespace ev_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BBoxArray_
{
  using Type = BBoxArray_<ContainerAllocator>;

  explicit BBoxArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit BBoxArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _bboxes_type =
    std::vector<ev_msgs::msg::BBox_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ev_msgs::msg::BBox_<ContainerAllocator>>>;
  _bboxes_type bboxes;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__bboxes(
    const std::vector<ev_msgs::msg::BBox_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ev_msgs::msg::BBox_<ContainerAllocator>>> & _arg)
  {
    this->bboxes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ev_msgs::msg::BBoxArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const ev_msgs::msg::BBoxArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ev_msgs::msg::BBoxArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ev_msgs::msg::BBoxArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ev_msgs::msg::BBoxArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ev_msgs::msg::BBoxArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ev_msgs::msg::BBoxArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ev_msgs::msg::BBoxArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ev_msgs::msg::BBoxArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ev_msgs::msg::BBoxArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ev_msgs__msg__BBoxArray
    std::shared_ptr<ev_msgs::msg::BBoxArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ev_msgs__msg__BBoxArray
    std::shared_ptr<ev_msgs::msg::BBoxArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BBoxArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->bboxes != other.bboxes) {
      return false;
    }
    return true;
  }
  bool operator!=(const BBoxArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BBoxArray_

// alias to use template instance with default allocator
using BBoxArray =
  ev_msgs::msg::BBoxArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ev_msgs

#endif  // EV_MSGS__MSG__DETAIL__B_BOX_ARRAY__STRUCT_HPP_
