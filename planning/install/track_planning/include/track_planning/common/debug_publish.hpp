/**
 * @file debug_publish.hpp
 * @brief RViz2 디버그 시각화를 위한 메시지 변환 유틸리티 (header-only)
 *
 * 내부 자료구조(Point2D, bool 등)를 ROS 2 메시지로 변환하여
 * /planning/debug/ 토픽으로 발행할 수 있게 해주는 헬퍼 함수들.
 *
 * 사용 예시:
 *   auto msg = to_path_msg(centerline, "base_link", now());
 *   pub_debug_->publish(std::move(msg));
 */
#ifndef TRACK_PLANNING__COMMON__DEBUG_PUBLISH_HPP_
#define TRACK_PLANNING__COMMON__DEBUG_PUBLISH_HPP_

#include "track_planning/common/types.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <string>
#include <vector>

namespace track_planning
{

/**
 * @brief Point2D 배열을 nav_msgs/Path 메시지로 변환
 *
 * RViz2의 Path 디스플레이로 폴리라인(corridor, centerline, path 등)을 시각화할 때 사용.
 * 각 점의 orientation은 기본값 (w=1, 회전 없음)으로 설정.
 *
 * @param pts       변환할 2D 점 배열
 * @param frame_id  좌표계 프레임 (보통 "base_link")
 * @param stamp     타임스탬프
 * @return nav_msgs::msg::Path 메시지
 */
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
    ps.pose.orientation.w = 1.0;  // 회전 없음 (단위 쿼터니언)
    msg.poses.push_back(ps);
  }
  return msg;
}

/// bool 값을 std_msgs/Bool 메시지로 변환
inline std_msgs::msg::Bool to_bool_msg(bool value)
{
  std_msgs::msg::Bool msg;
  msg.data = value;
  return msg;
}

}  // namespace track_planning

#endif  // TRACK_PLANNING__COMMON__DEBUG_PUBLISH_HPP_
