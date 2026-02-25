/**
 * @file debug_publish.hpp
 * @brief 디버그용 ROS 2 메시지 변환 헬퍼 함수
 *
 * Point2D 벡터를 nav_msgs/Path 메시지로 변환하는 등,
 * RViz2 시각화를 위한 메시지 생성 유틸리티를 제공한다.
 */
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

/**
 * @brief Point2D 벡터를 nav_msgs::msg::Path 메시지로 변환
 *
 * RViz2에서 경로를 시각화할 때 사용한다.
 * 각 Point2D의 z좌표는 0, orientation은 기본값(w=1)으로 설정된다.
 *
 * @param pts      변환할 2D 경로 점 목록
 * @param frame_id 좌표계 이름 (예: "base_link")
 * @param stamp    타임스탬프
 * @return nav_msgs::msg::Path ROS 2 경로 메시지
 */
inline nav_msgs::msg::Path to_path_msg(
  const std::vector<Point2D> & pts,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.poses.reserve(pts.size());  // 메모리 미리 확보 (재할당 방지)

  for (const auto & pt : pts) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg.header;           // 동일한 frame_id와 stamp 사용
    ps.pose.position.x = pt.x;
    ps.pose.position.y = pt.y;
    ps.pose.position.z = 0.0;         // 2D 플래너이므로 z = 0
    ps.pose.orientation.w = 1.0;       // 단위 쿼터니언 (회전 없음)
    msg.poses.push_back(ps);
  }
  return msg;
}

/**
 * @brief bool 값을 std_msgs::msg::Bool 메시지로 변환
 *
 * @param value 변환할 bool 값
 * @return std_msgs::msg::Bool ROS 2 메시지
 */
inline std_msgs::msg::Bool to_bool_msg(bool value)
{
  std_msgs::msg::Bool msg;
  msg.data = value;
  return msg;
}

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__COMMON__DEBUG_PUBLISH_HPP_
