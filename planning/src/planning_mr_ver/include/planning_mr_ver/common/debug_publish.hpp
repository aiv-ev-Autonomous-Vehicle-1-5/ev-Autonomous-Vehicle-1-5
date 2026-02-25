#ifndef PLANNING_MR_VER__COMMON__DEBUG_PUBLISH_HPP_
#define PLANNING_MR_VER__COMMON__DEBUG_PUBLISH_HPP_

#include "planning_mr_ver/common/types.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <string>
#include <vector>

namespace planning_mr_ver
{

inline nav_msgs::msg::Path to_path_msg(
  const std::vector<Point2D> & pts,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.poses.reserve(pts.size());

  for (const auto & pt : pts) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg.header;
    ps.pose.position.x = pt.x;
    ps.pose.position.y = pt.y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    msg.poses.push_back(ps);
  }
  return msg;
}

inline std_msgs::msg::Bool to_bool_msg(bool value)
{
  std_msgs::msg::Bool msg;
  msg.data = value;
  return msg;
}

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__COMMON__DEBUG_PUBLISH_HPP_
