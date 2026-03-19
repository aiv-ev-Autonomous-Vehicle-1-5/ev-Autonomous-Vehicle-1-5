/**
 * @file debug_publisher.cpp
 * @brief [Stage 7] 디버그 토픽 발행 유틸리티 구현
 *
 * costmap, obstacle_wall, curvature, branches, seeds 등의
 * 디버그 마커 생성/발행 함수 구현.
 * 모든 함수는 lazy publishing (구독자가 있을 때만 발행).
 *
 * [의존 관계]
 *   - debug_publisher.hpp: 함수 선언
 *   - types.hpp: CostmapResult, PostprocessResult, BranchInfo 등
 *   - geometry.hpp: dist(), cross2()
 */
#include "chaining_costmap_ver/nodes/debug_publisher.hpp"
#include "chaining_costmap_ver/common/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace chaining_costmap_ver
{

// ============================================================================
// costmap을 OccupancyGrid로 발행
// ============================================================================
void publish_debug_costmap(
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr & pub,
  const CostmapResult & costmap,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  if (pub->get_subscription_count() == 0 || !costmap.valid) return;

  auto grid_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  grid_msg->header.stamp = stamp;
  grid_msg->header.frame_id = frame_id;
  grid_msg->info.resolution = static_cast<float>(costmap.resolution);
  grid_msg->info.width = costmap.cols;
  grid_msg->info.height = costmap.rows;
  grid_msg->info.origin.position.x = costmap.origin_x;
  grid_msg->info.origin.position.y = costmap.origin_y;
  grid_msg->info.origin.orientation.w = 1.0;
  grid_msg->data.resize(costmap.rows * costmap.cols);
  for (int i = 0; i < costmap.rows * costmap.cols; ++i) {
    grid_msg->data[i] = static_cast<int8_t>(std::min(100.0, costmap.data[i]));
  }
  pub->publish(std::move(grid_msg));
}

// ============================================================================
// obstacle_cost 이상인 셀을 빨간색 CUBE_LIST로 발행
// ============================================================================
void publish_debug_obstacle_wall(
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub,
  const CostmapResult & costmap,
  double obstacle_cost,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  if (pub->get_subscription_count() == 0 || !costmap.valid) return;

  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker m;
  m.header.stamp = stamp;
  m.header.frame_id = frame_id;
  m.ns = "obstacle_wall";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::CUBE_LIST;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = costmap.resolution;
  m.scale.y = costmap.resolution;
  m.scale.z = 0.005;
  m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.6f;
  m.pose.orientation.w = 1.0;

  for (int r = 0; r < costmap.rows; ++r) {
    for (int c = 0; c < costmap.cols; ++c) {
      if (costmap.data[r * costmap.cols + c] >= obstacle_cost) {
        geometry_msgs::msg::Point pt;
        pt.x = costmap.origin_x + (c + 0.5) * costmap.resolution;
        pt.y = costmap.origin_y + (r + 0.5) * costmap.resolution;
        pt.z = -0.01;
        m.points.push_back(pt);
      }
    }
  }
  ma.markers.push_back(m);
  pub->publish(std::make_unique<visualization_msgs::msg::MarkerArray>(ma));
}

// ============================================================================
// 곡률 초과 지점을 노란색→빨간색 구체로 발행
// ============================================================================
void publish_debug_curvature(
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub,
  const PostprocessResult & pp_result,
  double r_min,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  if (pub->get_subscription_count() == 0 || pp_result.path.size() < 3) return;

  const double kappa_limit = (r_min > 1e-6) ? (1.0 / r_min) : 1e6;
  visualization_msgs::msg::MarkerArray ma;

  // DELETEALL
  visualization_msgs::msg::Marker del;
  del.header.stamp = stamp;
  del.header.frame_id = frame_id;
  del.ns = "curvature_exceed";
  del.id = -1;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  ma.markers.push_back(del);

  int marker_id = 0;
  for (size_t i = 0; i + 2 < pp_result.path.size(); ++i) {
    const auto & a = pp_result.path[i];
    const auto & b = pp_result.path[i + 1];
    const auto & c = pp_result.path[i + 2];

    const double ab = dist(a, b);
    const double bc = dist(b, c);
    const double ac = dist(a, c);
    const double denom = ab * bc * ac;
    if (denom < 1e-12) continue;

    const Point2D ba = b - a;
    const Point2D cb = c - b;
    const double kappa = 2.0 * std::abs(cross2(ba, cb)) / denom;

    if (kappa > kappa_limit) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame_id;
      m.ns = "curvature_exceed";
      m.id = marker_id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = b.x;
      m.pose.position.y = b.y;
      m.pose.position.z = 0.15;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.15;
      const double ratio = std::min((kappa / kappa_limit - 1.0) * 2.0, 1.0);
      m.color.r = 1.0f;
      m.color.g = static_cast<float>(1.0 - ratio);
      m.color.b = 0.0f;
      m.color.a = 0.9f;
      m.lifetime = rclcpp::Duration::from_seconds(0.2);
      ma.markers.push_back(m);
    }
  }
  pub->publish(std::make_unique<visualization_msgs::msg::MarkerArray>(ma));
}

// ============================================================================
// branch MarkerArray 생성 (좌/우 공용)
// ============================================================================
visualization_msgs::msg::MarkerArray make_branch_markers(
  const std::vector<BranchInfo> & branches,
  const std::vector<ChainPoint> & backbone,
  float r, float g, float b_color,
  const std::string & ns,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray ma;

  visualization_msgs::msg::Marker del;
  del.header.stamp = stamp;
  del.header.frame_id = frame_id;
  del.ns = ns;
  del.id = -1;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  ma.markers.push_back(del);

  int id = 0;
  for (const auto & br : branches) {
    if (br.points.empty() || br.parent_backbone_idx < 0 ||
        br.parent_backbone_idx >= static_cast<int>(backbone.size())) {
      continue;
    }

    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.ns = ns;
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.03;
    m.color.r = r; m.color.g = g; m.color.b = b_color; m.color.a = 0.8f;

    geometry_msgs::msg::Point pt;
    pt.x = backbone[br.parent_backbone_idx].x;
    pt.y = backbone[br.parent_backbone_idx].y;
    pt.z = 0.0;
    m.points.push_back(pt);
    for (const auto & bp : br.points) {
      pt.x = bp.x; pt.y = bp.y;
      m.points.push_back(pt);
    }
    ma.markers.push_back(m);
  }
  return ma;
}

// ============================================================================
// seeds & goals MarkerArray 생성
// ============================================================================
visualization_msgs::msg::MarkerArray make_seeds_markers(
  const DirectionChainResult & dc_result,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray ma;
  int id = 0;

  auto add_seed = [&](const SideResult & side, float r, float g, float b_color) {
    if (side.backbone.empty()) return;

    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id;
    m.ns = "seeds";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 0.15;
    m.pose.orientation.w = 1.0;

    // seed_back: 후방 체이닝 끝점 (backbone.front())
    m.color.r = r; m.color.g = g; m.color.b = b_color; m.color.a = 1.0f;
    m.pose.position.x = side.backbone.front().x;
    m.pose.position.y = side.backbone.front().y;
    ma.markers.push_back(m);

    // seed_start: find_seed()로 찾은 원래 시작점
    const int sp = side.seed_backbone_pos;
    if (sp >= 0 && sp < static_cast<int>(side.backbone.size())) {
      m.id = id++;
      m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f;  // 노랑
      m.pose.position.x = side.backbone[sp].x;
      m.pose.position.y = side.backbone[sp].y;
      ma.markers.push_back(m);
    }

    // seed_end: 전방 체이닝 끝점 (backbone.back())
    m.id = id++;
    m.color.r = 0.0f; m.color.g = 0.0f; m.color.b = 1.0f;  // 파랑
    m.pose.position.x = side.backbone.back().x;
    m.pose.position.y = side.backbone.back().y;
    ma.markers.push_back(m);
  };

  add_seed(dc_result.left, 0.0f, 1.0f, 0.0f);
  add_seed(dc_result.right, 1.0f, 0.0f, 0.0f);
  return ma;
}

}  // namespace chaining_costmap_ver
