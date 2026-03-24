/**
 * @file backbone_extractor.cpp
 * @brief DirectionChainer — Backbone 추출 + 비용함수 구현
 *
 * [포함 함수]
 *   - extract_backbone():      양방향(backward+forward) greedy chaining
 *                              reverse(backward) + [seed] + forward
 *   - chain_one_direction():   단방향 greedy chaining 헬퍼
 *                              (2-phase BBOX 최우선 탐색:
 *                               Phase 1 — d_max 범위 내 모든 bbox를 knn 없이
 *                               직접 전수 탐색 → 게이트 적용,
 *                               Phase 2 — bbox 후보 없으면 knn fallback)
 *   - compute_cost():          기본 비용함수 w(i,j)
 *   - compute_cost_prime():    확장 비용함수 w'(i,j) = w + λ·C_side
 *
 * [의존 관계]
 *   - geometry.hpp: dot2() — 2D 벡터 내적
 *   - types.hpp: ChainPoint, NodeOwner, StopReason
 *   - params.hpp: PlanningParams::Chainer
 */
#include "chaining_costmap_ver/chainer/direction_chainer.hpp"
#include "chaining_costmap_ver/common/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace chaining_costmap_ver
{

// ============================================================================
// 비용함수 w(i, j) — 기본 비용 계산
// ============================================================================
// w = α·C_d + β·C_a + γ·C_lat + δ·C_size
//
double DirectionChainer::compute_cost(
  const ChainPoint & pi,
  const ChainPoint & pj,
  const Point2D & v_i,
  const PlanningParams::Chainer & cp) const
{
  const double dx = pj.x - pi.x;
  const double dy = pj.y - pi.y;
  const double d = std::sqrt(dx * dx + dy * dy);

  // C_d: 거리 비용
  const double C_d = d / cp.d_max;

  // C_a: 방향 오차 비용
  double C_a = 0.0;
  if (d > 1e-9) {
    Point2D u_ij = {dx / d, dy / d};
    double cos_angle = dot2(v_i, u_ij);
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    double angle = std::acos(cos_angle);
    const double theta_max_rad = cp.forward_cone_deg * M_PI / 360.0;
    C_a = angle / theta_max_rad;
  }

  // C_lat: 횡오차 비용
  Point2D perp = {-v_i.y, v_i.x};
  double lat_offset = std::abs(dx * perp.x + dy * perp.y);
  const double C_lat = lat_offset / cp.lateral_gate;

  // C_size: 크기 변화 비용 (bbox↔bbox 전용)
  double C_size = 0.0;
  if (pi.type == PointType::BBOX && pj.type == PointType::BBOX) {
    double size_i = pi.size_x + pi.size_y;
    double size_j = pj.size_x + pj.size_y;
    const double eps = 1e-6;
    C_size = std::abs(size_j - size_i) / (size_i + eps);
  }

  return cp.alpha * C_d + cp.beta * C_a + cp.gamma * C_lat + cp.delta * C_size;
}

// ============================================================================
// w' = w + λ_side · C_side — 확장 비용함수
// ============================================================================
// 체인이 반대편 경계(중심선) 쪽으로 이동하는 것을 방지
//
double DirectionChainer::compute_cost_prime(
  const ChainPoint & pi,
  const ChainPoint & pj,
  const Point2D & v_i,
  bool is_left,
  const PlanningParams::Chainer & cp) const
{
  double w = compute_cost(pi, pj, v_i, cp);

  double C_side = 0.0;
  if (is_left) {
    C_side = std::max(0.0, pi.y - pj.y) / cp.lateral_gate;
  } else {
    C_side = std::max(0.0, pj.y - pi.y) / cp.lateral_gate;
  }

  return w + cp.lambda_side * C_side;
}

// ============================================================================
// 단방향 Greedy Chaining 헬퍼
// ============================================================================
// seed에서 주어진 초기 방향(init_dir)으로 한 방향만 greedy chaining.
//
// [2-phase BBOX 최우선 탐색]
//   Phase 1: d_max 범위 내 모든 bbox를 knn 없이 직접 전수 탐색 → G2+G3 게이트
//            (lane point가 많아도 bbox가 k개 제한에 밀리지 않음)
//   Phase 2: bbox 후보 없으면 기존 knn(k) → G1+G2+G3 게이트 → bbox-first 선택
//
std::vector<int> DirectionChainer::chain_one_direction(
  const std::vector<ChainPoint> & points,
  const std::vector<NodeOwner> & owner,
  int seed_idx,
  const Point2D & init_dir,
  bool is_left,
  std::unordered_set<int> & visited_set,
  int remaining_len,
  StopReason & stop_reason,
  const PlanningParams::Chainer & cp) const
{
  std::vector<int> chain;
  int current = seed_idx;
  Point2D v = init_dir;
  stop_reason = StopReason::MAX_LEN;
  const double cone_half_rad = cp.forward_cone_deg * M_PI / 360.0;

  while (static_cast<int>(chain.size()) < remaining_len) {
    std::vector<int> gated;
    bool had_candidates = false;
    const int n = static_cast<int>(points.size());
    const double d_max_sq = cp.d_max * cp.d_max;
    
    // G1 : 거리 게이트
    // ── Phase 1: BBOX 최우선 — d_max 범위 내 모든 bbox를 knn 없이 직접 탐색
    //    knn k개 제한 때문에 lane point에 밀려 bbox가 후보에서 빠지는 것을 방지
    for (int i = 0; i < n; ++i) {
      if (i == current) continue;
      if (points[i].type != PointType::BBOX) continue;

      const double dx = points[i].x - points[current].x;
      const double dy = points[i].y - points[current].y;
      if (dx * dx + dy * dy > d_max_sq) continue;

      if (owner[i] != NodeOwner::NONE) continue;
      had_candidates = true;
      if (visited_set.count(i)) continue;

      const double d = std::sqrt(dx * dx + dy * dy);
      if (d < 1e-9) continue;

      // G2: 전방 cone 게이트
      Point2D u_ij = {dx / d, dy / d};
      double cos_angle = dot2(v, u_ij);
      cos_angle = std::clamp(cos_angle, -1.0, 1.0);
      if (std::acos(cos_angle) > cone_half_rad) continue;

      // G3: 횡오차 게이트 (편측) — 안쪽(중심) 방향만 제한
      // perp = 방향벡터의 왼쪽 수직벡터 (왼쪽 +, 오른쪽 -)
      Point2D perp = {-v.y, v.x};
      double signed_lat = dx * perp.x + dy * perp.y;
      if (is_left  && signed_lat < -cp.lateral_gate) continue;  // 안쪽(오른쪽) 초과
      if (!is_left && signed_lat >  cp.lateral_gate) continue;  // 안쪽(왼쪽) 초과

      gated.push_back(i);
    }

    // ── Phase 2: bbox 후보가 없으면 기존 knn fallback (lane + bbox 혼합)
    if (gated.empty()) {
      auto neighbors = knn(points, current, cp.k);

      for (int j : neighbors) {
        if (owner[j] != NodeOwner::NONE) continue;
        had_candidates = true;
        if (visited_set.count(j)) continue;

        const double dx = points[j].x - points[current].x;
        const double dy = points[j].y - points[current].y;
        const double d = std::sqrt(dx * dx + dy * dy);

        // G1: 거리 게이트
        if (d > cp.d_max || d < 1e-9) continue;

        // G2: 전방 cone 게이트
        Point2D u_ij = {dx / d, dy / d};
        double cos_angle = dot2(v, u_ij);
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);
        if (std::acos(cos_angle) > cone_half_rad) continue;

        // G3: 횡오차 게이트 (편측) — 안쪽(중심) 방향만 제한
        Point2D perp = {-v.y, v.x};
        double signed_lat = dx * perp.x + dy * perp.y;
        if (is_left  && signed_lat < -cp.lateral_gate) continue;
        if (!is_left && signed_lat >  cp.lateral_gate) continue;

        gated.push_back(j);
      }

      // knn fallback에서도 bbox 우선 선택 유지
      if (!gated.empty()) {
        std::vector<int> bbox_gated;
        std::vector<int> lane_gated;
        for (int idx : gated) {
          if (points[idx].type == PointType::BBOX) {
            bbox_gated.push_back(idx);
          } else {
            lane_gated.push_back(idx);
          }
        }
        if (!bbox_gated.empty()) {
          gated = std::move(bbox_gated);
        }
      }
    }

    if (gated.empty()) {
      stop_reason = had_candidates ? StopReason::ALL_GATED
                                   : StopReason::NO_CANDIDATE;
      break;
    }

    // w' 최소 비용 선택
    int best = gated[0];
    double best_cost = compute_cost_prime(
      points[current], points[best], v, is_left, cp);

    for (size_t i = 1; i < gated.size(); ++i) {
      double c = compute_cost_prime(
        points[current], points[gated[i]], v, is_left, cp);
      if (c < best_cost) {
        best_cost = c;
        best = gated[i];
      }
    }

    // 이동 (진행 방향 갱신)
    const double dx = points[best].x - points[current].x;
    const double dy = points[best].y - points[current].y;
    const double d = std::sqrt(dx * dx + dy * dy);
    v = {dx / d, dy / d};

    visited_set.insert(best);
    chain.push_back(best);
    current = best;
  }

  return chain;
}

// ============================================================================
// Backbone 추출 — 양방향 Greedy Chaining
// ============================================================================
// seed에서 backward(-x) + forward(+x) 양방향 체이닝하여
// reverse(backward) + [seed] + forward → 최종 backbone.
// 전방/후방 각각 max_chain_len - 1 개까지 확장 가능.
//
std::vector<int> DirectionChainer::extract_backbone(
  const std::vector<ChainPoint> & points,
  const std::vector<NodeOwner> & owner,
  int seed_idx,
  bool is_left,
  StopReason & stop_reason_forward,
  int & seed_backbone_pos,
  const PlanningParams::Chainer & cp) const
{
  std::unordered_set<int> visited_set;
  visited_set.insert(seed_idx);

  const int max_extend = cp.max_chain_len - 1;

  // Backward pass (-x 방향)
  StopReason stop_reason_backward = StopReason::MAX_LEN;
  auto backward_chain = chain_one_direction(
    points, owner, seed_idx,
    {-1.0, 0.0}, is_left, visited_set,
    max_extend, stop_reason_backward, cp);

  // Forward pass (+x 방향)
  auto forward_chain = chain_one_direction(
    points, owner, seed_idx,
    {1.0, 0.0}, is_left, visited_set,
    max_extend, stop_reason_forward, cp);

  // reverse(backward) + [seed] + forward → backbone
  seed_backbone_pos = static_cast<int>(backward_chain.size());
  std::vector<int> backbone;
  backbone.reserve(backward_chain.size() + 1 + forward_chain.size());
  for (auto it = backward_chain.rbegin(); it != backward_chain.rend(); ++it) {
    backbone.push_back(*it);
  }
  backbone.push_back(seed_idx);
  backbone.insert(backbone.end(), forward_chain.begin(), forward_chain.end());
  return backbone;
}

}  // namespace chaining_costmap_ver
