/**
 * @file seed_selector.cpp
 * @brief DirectionChainer — Seed 선택 + kNN 탐색 구현
 *
 * [포함 함수]
 *   - find_seed(): 좌/우 시작점(seed) 선택 (단일 패스, bbox/lane 동일 가중치)
 *   - knn():       brute-force k-최근접 이웃 탐색
 *
 * [의존 관계]
 *   - types.hpp: ChainPoint
 *   - params.hpp: PlanningParams::Chainer
 */
#include "chaining_costmap_ver/chainer/direction_chainer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace chaining_costmap_ver
{

// ============================================================================
// Seed 선택 — 체이닝 시작점 결정
// ============================================================================
//
// [seed 선택 전략 — 단일 패스, bbox/lane 동일 가중치]
//   bbox와 lane 포인트를 동일하게 취급하여
//   (x ≥ -seed_rear_limit, side_seed_y 가드, d ≤ seed_max_dist) 조건 내
//   가장 가까운 점을 seed로 선택
//
int DirectionChainer::find_seed(
  const std::vector<ChainPoint> & points,
  bool is_left,
  const PlanningParams::Chainer & cp) const
{
  const char * side_str = is_left ? "LEFT" : "RIGHT";
  const int n = static_cast<int>(points.size());
  const double max_dist_sq = cp.seed_max_dist * cp.seed_max_dist;

  // 반대편 lane_side — lane 포인트 중 반대쪽 소속은 seed 후보에서 제외
  const LaneSide opp_side  = is_left ? LaneSide::RIGHT : LaneSide::LEFT;

  int best = -1;
  double best_dist_sq = std::numeric_limits<double>::max();
  int rear_skip = 0, side_skip = 0, dist_skip = 0, lane_side_skip = 0;

  for (int i = 0; i < n; ++i) {
    // lane_side 기반 필터: 반대편 차선 포인트 제외
    if (points[i].type == PointType::LANE && points[i].lane_side == opp_side) {
      ++lane_side_skip;
      continue;
    }

    if (points[i].x < -cp.seed_rear_limit) { ++rear_skip; continue; }

    if (is_left) {
      if (points[i].y < cp.side_seed_y) { ++side_skip; continue; }
    } else {
      if (points[i].y > -cp.side_seed_y) { ++side_skip; continue; }
    }

    const double d_sq = points[i].x * points[i].x +
                        points[i].y * points[i].y;
    if (d_sq > max_dist_sq) { ++dist_skip; continue; }
    if (d_sq < best_dist_sq) {
      best_dist_sq = d_sq;
      best = i;
    }
  }

  if (best >= 0) {
    std::fprintf(stderr, "[seed:%s] ✓ HIT → idx=%d (%.2f,%.2f) type=%s d=%.2f (rear_skip=%d side_skip=%d dist_skip=%d lane_side_skip=%d)\n",
      side_str, best, points[best].x, points[best].y,
      (points[best].type == PointType::BBOX ? "BBOX" : "LANE"),
      std::sqrt(best_dist_sq), rear_skip, side_skip, dist_skip, lane_side_skip);
  } else {
    std::fprintf(stderr, "[seed:%s] ✗ MISS — no seed found (rear_skip=%d side_skip=%d dist_skip=%d lane_side_skip=%d)\n",
      side_str, rear_skip, side_skip, dist_skip, lane_side_skip);
  }

  return best;
}

// ============================================================================
// kNN — brute-force k-최근접 이웃 탐색
// ============================================================================
//
// 경계점 수가 수십~수백 개로 적으므로 brute-force O(n)이 충분.
// partial_sort로 상위 k개만 정렬 → O(n·log(k)).
//
std::vector<int> DirectionChainer::knn(
  const std::vector<ChainPoint> & points,
  int query_idx,
  int k) const
{
  const int n = static_cast<int>(points.size());
  const double qx = points[query_idx].x;
  const double qy = points[query_idx].y;

  std::vector<std::pair<double, int>> dists;
  dists.reserve(n);
  for (int i = 0; i < n; ++i) {
    if (i == query_idx) continue;
    const double dx = points[i].x - qx;
    const double dy = points[i].y - qy;
    dists.push_back({dx * dx + dy * dy, i});
  }

  const int actual_k = std::min(k, static_cast<int>(dists.size()));
  std::partial_sort(dists.begin(), dists.begin() + actual_k, dists.end());

  std::vector<int> result;
  result.reserve(actual_k);
  for (int i = 0; i < actual_k; ++i) {
    result.push_back(dists[i].second);
  }
  return result;
}

}  // namespace chaining_costmap_ver
