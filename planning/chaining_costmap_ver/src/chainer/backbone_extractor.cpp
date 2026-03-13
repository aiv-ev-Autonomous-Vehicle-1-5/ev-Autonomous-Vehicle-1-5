/**
 * @file backbone_extractor.cpp
 * @brief DirectionChainer — Backbone 추출 + 비용함수 구현
 *
 * [포함 함수]
 *   - extract_backbone():      양방향(forward+backward) greedy chaining
 *   - chain_one_direction():   단방향 greedy chaining 헬퍼
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

  // C_size: 크기 변화 비용 (콘↔콘 전용)
  double C_size = 0.0;
  if (pi.type == PointType::CONE && pj.type == PointType::CONE) {
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
// G1(거리) + G2(콘) + G3(횡오차) 게이트 적용.
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
    auto neighbors = knn(points, current, cp.k);

    // owner 필터 + 3개 게이트 적용
    std::vector<int> gated;
    bool had_candidates = false;

    for (int j : neighbors) {
      if (owner[j] != NodeOwner::NONE) continue;
      had_candidates = true;
      if (visited_set.count(j)) continue;

      const double dx = points[j].x - points[current].x;
      const double dy = points[j].y - points[current].y;
      const double d = std::sqrt(dx * dx + dy * dy);

      // G1: 거리 게이트
      if (d > cp.d_max || d < 1e-9) continue;

      // G2: 콘 게이트
      Point2D u_ij = {dx / d, dy / d};
      double cos_angle = dot2(v, u_ij);
      cos_angle = std::clamp(cos_angle, -1.0, 1.0);
      if (std::acos(cos_angle) > cone_half_rad) continue;

      // G3: 횡오차 게이트
      Point2D perp = {-v.y, v.x};
      double lat = std::abs(dx * perp.x + dy * perp.y);
      if (lat > cp.lateral_gate) continue;

      gated.push_back(j);
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
// seed에서 forward(+x) + backward(-x) 양쪽으로 체이닝하여
// reverse(backward) + [seed] + forward → 최종 backbone.
//
std::vector<int> DirectionChainer::extract_backbone(
  const std::vector<ChainPoint> & points,
  const std::vector<NodeOwner> & owner,
  int seed_idx,
  bool is_left,
  StopReason & stop_reason_forward,
  StopReason & stop_reason_backward,
  const PlanningParams::Chainer & cp) const
{
  std::unordered_set<int> visited_set;
  visited_set.insert(seed_idx);

  const int max_extend = cp.max_chain_len - 1;

  // Forward pass
  auto forward_chain = chain_one_direction(
    points, owner, seed_idx,
    {1.0, 0.0}, is_left, visited_set,
    max_extend, stop_reason_forward, cp);

  // Backward pass
  const int backward_budget = max_extend - static_cast<int>(forward_chain.size());
  if (backward_budget > 0) {
    auto backward_chain = chain_one_direction(
      points, owner, seed_idx,
      {-1.0, 0.0}, is_left, visited_set,
      backward_budget, stop_reason_backward, cp);

    // 결합: reverse(backward) + [seed] + forward
    std::vector<int> backbone;
    backbone.reserve(backward_chain.size() + 1 + forward_chain.size());
    for (auto it = backward_chain.rbegin(); it != backward_chain.rend(); ++it) {
      backbone.push_back(*it);
    }
    backbone.push_back(seed_idx);
    backbone.insert(backbone.end(), forward_chain.begin(), forward_chain.end());
    return backbone;
  }

  // backward 예산이 0 이하이면 forward만으로 구성
  stop_reason_backward = StopReason::MAX_LEN;
  std::vector<int> backbone;
  backbone.reserve(1 + forward_chain.size());
  backbone.push_back(seed_idx);
  backbone.insert(backbone.end(), forward_chain.begin(), forward_chain.end());
  return backbone;
}

}  // namespace chaining_costmap_ver
