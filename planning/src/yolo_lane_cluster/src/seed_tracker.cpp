/**
 * @file seed_tracker.cpp
 * @brief 시드 매칭 및 업데이트 로직
 *
 * 각 시드(좌/우)의 직사각형 탐색 영역 내에서
 * 가장 가까운 클러스터를 찾아 매칭한다.
 *
 * 시드 추적:
 *   - 매칭 성공 시: 시드 중심 x = 매칭된 클러스터의 x_min (y는 초기값 유지)
 *   - 매칭 실패 시: 시드 중심 유지 (마지막 위치 보존)
 */
#include "yolo_lane_cluster/yolo_lane_cluster_node.hpp"

#include <cmath>
#include <limits>

namespace yolo_lane_cluster
{

int YoloLaneClusterNode::match_cluster_to_seed(
  const SeedState & seed,
  const ev_msgs::msg::LaneBoundaryArray & msg,
  int exclude_idx) const
{
  double sx = seed.center_x;
  double sy = seed.center_y;
  double half_w = params_.search_rect_width  / 2.0;
  double half_h = params_.search_rect_height / 2.0;

  double best_dist_sq = std::numeric_limits<double>::max();
  int best_idx = -1;

  for (int i = 0; i < static_cast<int>(msg.boundaries.size()); ++i) {
    if (i == exclude_idx) continue;

    const auto & bd = msg.boundaries[i];
    for (const auto & pt : bd.points) {
      // 직사각형 범위 검사
      if (pt.x < sx - half_h || pt.x > sx + half_h) continue;
      if (pt.y < sy - half_w || pt.y > sy + half_w) continue;

      // 시드 중심까지 거리 (제곱)
      double dx = pt.x - sx;
      double dy = pt.y - sy;
      double d_sq = dx * dx + dy * dy;
      if (d_sq < best_dist_sq) {
        best_dist_sq = d_sq;
        best_idx = i;
      }
    }
  }

  return best_idx;
}

void YoloLaneClusterNode::update_seed(
  SeedState & seed,
  const ev_msgs::msg::LaneBoundary & boundary)
{
  if (boundary.points.empty()) return;

  // x_min 찾기 — costmap에서 차량에 가장 가까운(뒤쪽) 지점
  double x_min = std::numeric_limits<double>::max();
  for (const auto & pt : boundary.points) {
    if (pt.x < x_min) {
      x_min = pt.x;
    }
  }

  seed.center_x = x_min;
  // y는 초기 오프셋 유지 (좌/우 정체성 보존)
  seed.has_match = true;
}

void YoloLaneClusterNode::reset_seed(SeedState & seed, LaneSide side)
{
  seed.center_x = params_.seed_init_x;
  seed.center_y = (side == LaneSide::LEFT)
                    ? params_.seed_left_y
                    : params_.seed_right_y;
  seed.has_match = false;
}

}  // namespace yolo_lane_cluster
