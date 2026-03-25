/**
 * @file seed_selector.cpp
 * @brief DirectionChainer — Seed 선택 + kNN 탐색 구현
 *
 * [포함 함수]
 *   - find_seed(): 좌/우 시작점(seed) 선택 (2-pass bbox 우선)
 *   - knn():       brute-force k-최근접 이웃 탐색
 *
 * [의존 관계]
 *   - types.hpp: ChainPoint
 *   - params.hpp: PlanningParams::Chainer
 */
#include "chaining_costmap_ver/chainer/direction_chainer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace chaining_costmap_ver
{

// ============================================================================
// Seed 선택 — 체이닝 시작점 결정
// ============================================================================
//
// [seed 선택 전략 — 2-pass bbox 우선]
//   Pass 1: bbox만 탐색 (x ≥ -seed_rear_limit, side_seed_y 가드, d ≤ seed_bbox_max_dist)
//           → 조건 만족 bbox 중 가장 가까운 것 반환
//   Pass 2: Pass 1 실패 시 bbox+lane 전체에서 가장 가까운 점 (기존 로직)
//
int DirectionChainer::find_seed(
  const std::vector<ChainPoint> & points,
  bool is_left,
  const PlanningParams::Chainer & cp) const
{
  const int n = static_cast<int>(points.size());
  const double bbox_max_sq = cp.seed_bbox_max_dist * cp.seed_bbox_max_dist;

  // ── Pass 1: bbox 우선 탐색 (seed_bbox_max_dist 이내) ──
  int best_bbox = -1;
  double best_bbox_sq = std::numeric_limits<double>::max();

  for (int i = 0; i < n; ++i) {
    if (points[i].x < -cp.seed_rear_limit) continue;
    if (points[i].type != PointType::BBOX) continue;

    if (is_left) {
      if (points[i].y < cp.side_seed_y) continue;
    } else {
      if (points[i].y > -cp.side_seed_y) continue;
    }

    const double d_sq = points[i].x * points[i].x +
                        points[i].y * points[i].y;
    if (d_sq > bbox_max_sq) continue;

    if (d_sq < best_bbox_sq) {
      best_bbox_sq = d_sq;
      best_bbox = i;
    }
  }

  if (best_bbox >= 0) return best_bbox;

  // ── Pass 2: bbox 없으면 전체(bbox+lane)에서 가장 가까운 점 ──
  int best = -1;
  double best_dist_sq = std::numeric_limits<double>::max();

  for (int i = 0; i < n; ++i) {
    if (points[i].x < -cp.seed_rear_limit) continue;

    if (is_left) {
      if (points[i].y < cp.side_seed_y) continue;
    } else {
      if (points[i].y > -cp.side_seed_y) continue;
    }

    const double d_sq = points[i].x * points[i].x +
                        points[i].y * points[i].y;
    if (d_sq < best_dist_sq) {
      best_dist_sq = d_sq;
      best = i;
    }
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
