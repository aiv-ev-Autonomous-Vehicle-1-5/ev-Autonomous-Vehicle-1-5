#ifndef TRACK_PLANNING__COMMON__DEBUG_PUBLISH_HPP_
#define TRACK_PLANNING__COMMON__DEBUG_PUBLISH_HPP_

#include "track_planning/common/types.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <vector>

namespace track_planning
{

/// Convert a vector of Point2D to nav_msgs/Path (for RViz visualization)
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

/// Create a Bool message
inline std_msgs::msg::Bool to_bool_msg(bool value)
{
  std_msgs::msg::Bool msg;
  msg.data = value;
  return msg;
}

/// Create a String message
inline std_msgs::msg::String to_string_msg(const std::string & value)
{
  std_msgs::msg::String msg;
  msg.data = value;
  return msg;
}

}  // namespace track_planning

#endif  // TRACK_PLANNING__COMMON__DEBUG_PUBLISH_HPP_
