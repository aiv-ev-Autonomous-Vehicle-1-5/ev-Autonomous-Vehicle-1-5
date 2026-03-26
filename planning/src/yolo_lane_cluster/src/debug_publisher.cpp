/**
 * @file debug_publisher.cpp
 * @brief 디버그 시각화 — lazy Marker 토픽
 *
 * 구독자가 있을 때만 발행:
 *   - 실제 차선 points (파란색)
 *   - 가상 차선 points (노란색)
 *   - 왼쪽 시드 위치 (초록색 구)
 *   - 오른쪽 시드 위치 (빨간색 구)
 *   - 시드 탐색 직사각형 (LINE_STRIP)
 */
#include "yolo_lane_cluster/yolo_lane_cluster_node.hpp"

namespace yolo_lane_cluster
{

namespace
{

/// 시드 위치를 SPHERE 마커로 생성
visualization_msgs::msg::Marker make_seed_marker(
  const SeedState & seed, int id,
  float r, float g, float b,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::Marker m;
  m.header    = header;
  m.ns        = "seeds";
  m.id        = id;
  m.type      = visualization_msgs::msg::Marker::SPHERE;
  m.action    = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = seed.center_x;
  m.pose.position.y = seed.center_y;
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

/// 시드 탐색 직사각형을 LINE_STRIP 마커로 생성
visualization_msgs::msg::Marker make_rect_marker(
  const SeedState & seed, int id,
  double half_w, double half_h,
  float r, float g, float b,
  const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::Marker m;
  m.header    = header;
  m.ns        = "search_rects";
  m.id        = id;
  m.type      = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action    = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x   = 0.05;
  m.color.r   = r;
  m.color.g   = g;
  m.color.b   = b;
  m.color.a   = 0.5f;
  m.lifetime  = rclcpp::Duration::from_seconds(0.2);

  double cx = seed.center_x, cy = seed.center_y;
  auto pt = [](double x, double y) {
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = 0.05;
    return p;
  };
  m.points.push_back(pt(cx - half_h, cy - half_w));
  m.points.push_back(pt(cx + half_h, cy - half_w));
  m.points.push_back(pt(cx + half_h, cy + half_w));
  m.points.push_back(pt(cx - half_h, cy + half_w));
  m.points.push_back(pt(cx - half_h, cy - half_w));  // close
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
  bool /*left_matched*/, bool /*right_matched*/,
  bool /*left_virtual*/, bool /*right_virtual*/)
{
  // lazy: 구독자 없으면 스킵
  if (debug_pub_->get_subscription_count() == 0) return;

  visualization_msgs::msg::MarkerArray ma;
  const auto & hdr = output.header;
  double half_w = params_.search_rect_width  / 2.0;
  double half_h = params_.search_rect_height / 2.0;

  // 시드 마커
  ma.markers.push_back(make_seed_marker(left_seed_,  0, 1.0f, 0.5f, 0.0f, hdr));  // 주황
  ma.markers.push_back(make_seed_marker(right_seed_, 1, 0.3f, 0.7f, 1.0f, hdr));  // 하늘색

  // 탐색 직사각형
  ma.markers.push_back(make_rect_marker(left_seed_,  0, half_w, half_h,
                                        1.0f, 0.5f, 0.0f, hdr));
  ma.markers.push_back(make_rect_marker(right_seed_, 1, half_w, half_h,
                                        0.3f, 0.7f, 1.0f, hdr));

  // lane points
  int pt_id = 0;
  for (const auto & bd : output.boundaries) {
    bool is_virtual = (bd.lane_id == -1);
    if (is_virtual) {
      // 가상 차선: 노란색
      ma.markers.push_back(make_points_marker(
        bd, pt_id++, "virtual_lane", 1.0f, 1.0f, 0.0f, hdr));
    } else {
      // 실제 차선: 파란색
      ma.markers.push_back(make_points_marker(
        bd, pt_id++, "real_lane", 0.0f, 0.5f, 1.0f, hdr));
    }
  }

  debug_pub_->publish(ma);
}

}  // namespace yolo_lane_cluster
