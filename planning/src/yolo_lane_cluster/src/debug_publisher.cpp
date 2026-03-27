/**
 * @file debug_publisher.cpp
 * @brief 디버그 시각화 — lazy Marker 토픽
 *
 * 구독자가 있을 때만 발행:
 *   - 실제 차선 points (파란색)
 *   - 가상 차선 points (노란색)
 *   - 왼쪽 시드 위치 (주황색 구) — 추적 중인 seed 위치
 *   - 오른쪽 시드 위치 (하늘색 구) — 추적 중인 seed 위치
 */
#include "yolo_lane_cluster/yolo_lane_cluster_node.hpp"

namespace yolo_lane_cluster
{

namespace
{

/// 고정 seed 위치를 SPHERE 마커로 생성
visualization_msgs::msg::Marker make_seed_marker(
  double sx, double sy, int id,
  float r, float g, float b,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::Marker m;
  m.header    = header;
  m.ns        = "seeds";
  m.id        = id;
  m.type      = visualization_msgs::msg::Marker::SPHERE;
  m.action    = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = sx;
  m.pose.position.y = sy;
  m.pose.position.z = 0.1;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.3;
  m.scale.y = 0.3;
  m.scale.z = 0.3;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 0.8f;
  m.lifetime = rclcpp::Duration::from_seconds(0.2);
  return m;
}

/// lane points를 POINTS 마커로 생성
visualization_msgs::msg::Marker make_points_marker(
  const ev_msgs::msg::LaneBoundary & bd, int id,
  const std::string & ns,
  float r, float g, float b,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::Marker m;
  m.header    = header;
  m.ns        = ns;
  m.id        = id;
  m.type      = visualization_msgs::msg::Marker::POINTS;
  m.action    = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x   = 0.08;
  m.scale.y   = 0.08;
  m.color.r   = r;
  m.color.g   = g;
  m.color.b   = b;
  m.color.a   = 1.0f;
  m.lifetime  = rclcpp::Duration::from_seconds(0.2);

  for (const auto & pt : bd.points) {
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.0;
    m.points.push_back(p);
  }
  return m;
}

}  // anonymous namespace

void YoloLaneClusterNode::publish_debug_markers(
  const ev_msgs::msg::LaneBoundaryArray & output,
  bool /*left_chained*/, bool /*right_chained*/,
  bool /*left_virtual*/, bool /*right_virtual*/)
{
  // lazy: 구독자 없으면 스킵
  if (debug_pub_->get_subscription_count() == 0) return;

  visualization_msgs::msg::MarkerArray ma;
  const auto & hdr = output.header;

  // seed 마커 (추적 위치)
  ma.markers.push_back(make_seed_marker(
    left_seed_.center_x, left_seed_.center_y, 0, 1.0f, 0.5f, 0.0f, hdr));   // 주황
  ma.markers.push_back(make_seed_marker(
    right_seed_.center_x, right_seed_.center_y, 1, 0.3f, 0.7f, 1.0f, hdr));  // 하늘색

  // lane points
  int pt_id = 0;
  for (const auto & bd : output.boundaries) {
    bool is_virtual = (bd.lane_id == -1);
    if (is_virtual) {
      // 가상 차선: 노란색
      ma.markers.push_back(make_points_marker(
        bd, pt_id++, "virtual_lane", 1.0f, 1.0f, 0.0f, hdr));
    } else {
      // 실제 차선 (chained backbone): 파란색
      ma.markers.push_back(make_points_marker(
        bd, pt_id++, "real_lane", 0.0f, 0.5f, 1.0f, hdr));
    }
  }

  debug_pub_->publish(ma);
}

}  // namespace yolo_lane_cluster
