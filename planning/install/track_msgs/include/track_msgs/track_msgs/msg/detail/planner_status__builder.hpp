// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from track_msgs:msg/PlannerStatus.idl
// generated code does not contain a copyright notice

#ifndef TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__BUILDER_HPP_
#define TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "track_msgs/msg/detail/planner_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace track_msgs
{

namespace msg
{

namespace builder
{

class Init_PlannerStatus_reason
{
public:
  explicit Init_PlannerStatus_reason(::track_msgs::msg::PlannerStatus & msg)
  : msg_(msg)
  {}
  ::track_msgs::msg::PlannerStatus reason(::track_msgs::msg::PlannerStatus::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::track_msgs::msg::PlannerStatus msg_;
};

class Init_PlannerStatus_status
{
public:
  explicit Init_PlannerStatus_status(::track_msgs::msg::PlannerStatus & msg)
  : msg_(msg)
  {}
  Init_PlannerStatus_reason status(::track_msgs::msg::PlannerStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PlannerStatus_reason(msg_);
  }

private:
  ::track_msgs::msg::PlannerStatus msg_;
};

class Init_PlannerStatus_header
{
public:
  Init_PlannerStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlannerStatus_status header(::track_msgs::msg::PlannerStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PlannerStatus_status(msg_);
  }

private:
  ::track_msgs::msg::PlannerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::track_msgs::msg::PlannerStatus>()
{
  return track_msgs::msg::builder::Init_PlannerStatus_header();
}

}  // namespace track_msgs

#endif  // TRACK_MSGS__MSG__DETAIL__PLANNER_STATUS__BUILDER_HPP_
