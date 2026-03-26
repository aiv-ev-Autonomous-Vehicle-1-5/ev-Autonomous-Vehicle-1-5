/**
 * @file debug_publish.hpp
 * @brief 디버그용 ROS 2 메시지 변환 헬퍼 함수
 *
 * 내부 자료구조(Point2D 등)를 ROS 2 표준 메시지(nav_msgs::msg::Path)로
 * 변환하는 인라인 유틸리티 함수 모음.
 *
 * ── 사용 위치 ──
 *   lc_planner_node.cpp  Stage 7 (Publish) 에서 호출된다.
 *     - to_path_msg() : 최종 경로, raw 경로, 좌/우 backbone 체인을
 *                       nav_msgs/Path로 변환하여 RViz2에서 시각화.
 *
 * ── 왜 inline 인가? ──
 *   헤더 전용(header-only) 유틸이므로 .cpp 없이 여러 TU에서 include해도
 *   ODR(One Definition Rule) 위반 없이 링크된다.
 */
#ifndef CHAINING_COSTMAP_VER__COMMON__DEBUG_PUBLISH_HPP_
#define CHAINING_COSTMAP_VER__COMMON__DEBUG_PUBLISH_HPP_

#include "chaining_costmap_ver/common/types.hpp"

// ── ROS 2 메시지 헤더 ──
// geometry_msgs/PoseStamped : Path 안에 들어가는 개별 pose 요소
// nav_msgs/Path            : PoseStamped 배열 → RViz2 "Path" 디스플레이로 시각화 가능
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace chaining_costmap_ver
{

// ============================================================================
// to_path_msg  — Point2D 벡터 → nav_msgs::msg::Path 변환
// ============================================================================
/**
 * @brief 내부 Point2D 벡터를 nav_msgs::msg::Path 메시지로 변환한다.
 *
 * ── 변환 규칙 ──
 *   Point2D(x, y)  →  PoseStamped.pose.position(x, y, z=0)
 *   orientation은 단위 쿼터니언(w=1)으로 채운다.
 *   (경로의 "방향"은 별도로 yaw를 계산하지 않고, 시각화 목적으로만 쓰인다.)
 *
 * ── nav_msgs::msg::Path 메시지 구조 ──
 *   header:
 *     frame_id  : 좌표계 (보통 "base_link")
 *     stamp     : 현재 타임스탬프
 *   poses[]:     PoseStamped 배열
 *     각 요소는 header(동일) + pose(position + orientation)
 *
 * ── 호출 예시 (lc_planner_node.cpp) ──
 *   1) 최종 경로 퍼블리시:
 *        to_path_msg(pp_result.path, frame_id, stamp)
 *        → pub_path_ 로 퍼블리시 → RViz2 "Path" 디스플레이에서 시각화
 *   2) raw path (후처리 전 원본 경로):
 *        to_path_msg(raw_path, frame_id, stamp)
 *        → pub_dbg_raw_path_ 로 퍼블리시 (구독자가 있을 때만)
 *   3) 좌/우 backbone 체인:
 *        backbone의 ChainPoint → Point2D 변환 후 to_path_msg() 호출
 *        → pub_dbg_left_chain_ / pub_dbg_right_chain_ 으로 퍼블리시
 *
 * @param pts       변환할 Point2D 벡터 (경로 점들)
 * @param frame_id  좌표계 이름 (예: "base_link")
 * @param stamp     ROS 타임스탬프 (메시지 헤더에 들어감)
 * @return          변환된 nav_msgs::msg::Path 메시지
 */
inline nav_msgs::msg::Path to_path_msg(
  const std::vector<Point2D> & pts,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  // Path 메시지 생성 및 헤더 설정
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id;   // 좌표계 (base_link 등)
  msg.header.stamp = stamp;         // 현재 시각
  msg.poses.reserve(pts.size());    // 미리 메모리 할당 (push_back 시 재할당 방지)

  // 각 Point2D를 PoseStamped로 변환하여 poses 배열에 추가
  for (const auto & pt : pts) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg.header;          // Path 전체와 동일한 헤더 공유
    ps.pose.position.x = pt.x;      // [m] 전방(+) / 후방(-)
    ps.pose.position.y = pt.y;      // [m] 좌측(+) / 우측(-)
    ps.pose.position.z = 0.0;       // 2D 플래너이므로 z=0 고정
    ps.pose.orientation.w = 1.0;    // 단위 쿼터니언 (회전 없음)
    msg.poses.push_back(ps);
  }
  return msg;
}


// ============================================================================
// to_points_marker  — Point2D 벡터 → visualization_msgs::msg::Marker (POINTS) 변환
// ============================================================================
/**
 * @brief Point2D 벡터를 POINTS 타입 Marker로 변환한다.
 *
 * RViz2에서 각 점을 개별 구체로 표시하므로 점 개수를 직관적으로 파악 가능.
 * Path 타입과 달리 점 사이를 직선으로 연결하지 않는다.
 *
 * @param pts       변환할 Point2D 벡터
 * @param frame_id  좌표계 이름
 * @param stamp     ROS 타임스탬프
 * @param ns        마커 namespace (토픽 구분용)
 * @param r,g,b,a   마커 색상 (0.0~1.0)
 * @param scale     점 크기 [m]
 */
inline visualization_msgs::msg::Marker to_points_marker(
  const std::vector<Point2D> & pts,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & ns = "points",
  float r = 1.0f, float g = 1.0f, float b = 1.0f, float a = 1.0f,
  double scale = 0.08)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::POINTS;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = scale;
  m.scale.y = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
  m.lifetime = rclcpp::Duration::from_seconds(0.0);
  m.points.reserve(pts.size());
  for (const auto & pt : pts) {
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.0;
    m.points.push_back(p);
  }
  return m;
}

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__COMMON__DEBUG_PUBLISH_HPP_
