/**
 * @file lane_chainer.cpp
 * @brief Backbone chaining — chaining_costmap_ver의 greedy chaining 알고리즘 간소화 이식
 *
 * 카메라 클러스터 포인트 풀에서 LEFT/RIGHT backbone을 추출한다.
 *
 * [포함 함수]
 *   - find_seed():           고정 seed 좌표에 가장 가까운 포인트 인덱스
 *   - extract_backbone():    [seed] + forward greedy chaining
 *   - chain_one_direction(): 단방향 greedy chaining (G1+G2+G3 게이트 + 클러스터 락)
 *   - compute_cost():        기본 비용 w = α·C_d + β·C_a + γ·C_lat
 *   - compute_cost_prime():  확장 비용 w' = w + λ·C_side
 *   - resolve_overlaps():    좌/우 backbone 중복 해소 backtracking
 *   - compute_backtrack_cost(): 3-node 윈도우 곡률+거리 비용
 *   - rechain_from():        truncate 후 재chaining
 *   - backbone_to_boundary(): backbone → LaneBoundary 변환
 */
#include "yolo_lane_cluster/yolo_lane_cluster_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace yolo_lane_cluster
{

// ============================================================================
// find_seed — 고정 seed 좌표에 가장 가까운 포인트
// ============================================================================
int YoloLaneClusterNode::find_seed(
  const std::vector<LanePoint> & points,
  double seed_x, double seed_y) const
{
  double best_dist_sq = std::numeric_limits<double>::max();
  int best_idx = -1;

  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    double dx = points[i].x - seed_x;
    double dy = points[i].y - seed_y;
    double d_sq = dx * dx + dy * dy;
    if (d_sq < best_dist_sq) {
      best_dist_sq = d_sq;
      best_idx = i;
    }
  }

  return best_idx;
}

// ============================================================================
// compute_cost — 기본 비용함수 w = α·C_d + β·C_a + γ·C_lat
// ============================================================================
double YoloLaneClusterNode::compute_cost(
  const LanePoint & pi,
  const LanePoint & pj,
  double vx, double vy) const
{
  const auto & cp = params_.chainer;

  const double dx = pj.x - pi.x;
  const double dy = pj.y - pi.y;
  const double d = std::sqrt(dx * dx + dy * dy);

  // C_d: 거리 비용
  const double C_d = d / cp.d_max;

  // C_a: 방향 오차 비용
  double C_a = 0.0;
  if (d > 1e-9) {
    double ux = dx / d, uy = dy / d;
    double cos_angle = vx * ux + vy * uy;
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    double angle = std::acos(cos_angle);
    const double theta_max_rad = cp.forward_cone_deg * M_PI / 360.0;
    C_a = angle / theta_max_rad;
  }

  // C_lat: 횡오차 비용 — 진행 방향 수직 성분
  double perp_x = -vy, perp_y = vx;
  double lat_offset = std::abs(dx * perp_x + dy * perp_y);
  const double C_lat = lat_offset / cp.lateral_gate;

  return cp.alpha * C_d + cp.beta * C_a + cp.gamma * C_lat;
}

// ============================================================================
// compute_cost_prime — 확장 비용함수 w' = w + λ·C_side
// ============================================================================
double YoloLaneClusterNode::compute_cost_prime(
  const LanePoint & pi,
  const LanePoint & pj,
  double vx, double vy,
  bool is_left) const
{
  double w = compute_cost(pi, pj, vx, vy);

  const auto & cp = params_.chainer;
  double C_side = 0.0;
  if (is_left) {
    // 왼쪽 chain: y 감소(안쪽) 시 페널티
    C_side = std::max(0.0, pi.y - pj.y) / cp.lateral_gate;
  } else {
    // 오른쪽 chain: y 증가(안쪽) 시 페널티
    C_side = std::max(0.0, pj.y - pi.y) / cp.lateral_gate;
  }

  return w + cp.lambda_side * C_side;
}

// ============================================================================
// chain_one_direction — 단방향 greedy chaining
// ============================================================================
// G1(거리) + G2(forward cone) + G3(편측 lateral) + 클러스터 락
//
std::vector<int> YoloLaneClusterNode::chain_one_direction(
  const std::vector<LanePoint> & points,
  const std::vector<bool> & owner,
  int seed_idx,
  double vx, double vy,
  bool is_left,
  std::unordered_set<int> & visited_set,
  int remaining_len) const
{
  const auto & cp = params_.chainer;
  std::vector<int> chain;
  int current = seed_idx;
  const double cone_half_rad = cp.forward_cone_deg * M_PI / 360.0;
  const double d_max_sq = cp.d_max * cp.d_max;

  // 클러스터 락: 특정 클러스터 진입 시 해당 클러스터만 후보
  int32_t locked_cluster_label = -1;

  while (static_cast<int>(chain.size()) < remaining_len) {
    std::vector<int> gated;
    const int n = static_cast<int>(points.size());

    for (int i = 0; i < n; ++i) {
      if (i == current) continue;

      // 클러스터 락: 해당 클러스터 포인트만 후보
      if (locked_cluster_label >= 0) {
        if (points[i].label != locked_cluster_label) continue;
      }

      const double dx = points[i].x - points[current].x;
      const double dy = points[i].y - points[current].y;

      // G1: 거리 게이트
      if (dx * dx + dy * dy > d_max_sq) continue;

      // owner 체크 (이미 상대 chain에 속한 포인트 제외 — rechain 시)
      if (owner[i]) continue;
      if (visited_set.count(i)) continue;

      const double d = std::sqrt(dx * dx + dy * dy);
      if (d < 1e-9) continue;

      // G2: 전방 cone 게이트
      double ux = dx / d, uy = dy / d;
      double cos_angle = vx * ux + vy * uy;
      cos_angle = std::clamp(cos_angle, -1.0, 1.0);
      if (std::acos(cos_angle) > cone_half_rad) continue;

      // G3: 횡오차 게이트 (편측)
      double perp_x = -vy, perp_y = vx;
      double signed_lat = dx * perp_x + dy * perp_y;
      if (is_left  && signed_lat < -cp.lateral_gate) continue;
      if (!is_left && signed_lat >  cp.lateral_gate) continue;

      gated.push_back(i);
    }

    if (gated.empty()) {
      // 클러스터 락 중 후보 소진 → 락 해제, 정상 탐색 복귀
      if (locked_cluster_label >= 0) {
        locked_cluster_label = -1;
        continue;
      }
      break;
    }

    // w' 최소 비용 선택
    int best = gated[0];
    double best_cost = compute_cost_prime(
      points[current], points[best], vx, vy, is_left);

    for (size_t i = 1; i < gated.size(); ++i) {
      double c = compute_cost_prime(
        points[current], points[gated[i]], vx, vy, is_left);
      if (c < best_cost) {
        best_cost = c;
        best = gated[i];
      }
    }

    // 클러스터 락 진입: 새 클러스터 포인트 선택 시 해당 클러스터 잠금
    if (locked_cluster_label < 0) {
      locked_cluster_label = points[best].label;
    }

    // 이동 (진행 방향 갱신)
    const double dx = points[best].x - points[current].x;
    const double dy = points[best].y - points[current].y;
    const double d = std::sqrt(dx * dx + dy * dy);
    if (d > 1e-9) {
      vx = dx / d;
      vy = dy / d;
    }

    visited_set.insert(best);
    chain.push_back(best);
    current = best;
  }

  return chain;
}

// ============================================================================
// extract_backbone — [seed] + forward greedy chaining
// ============================================================================
std::vector<int> YoloLaneClusterNode::extract_backbone(
  const std::vector<LanePoint> & points,
  int seed_idx,
  bool is_left) const
{
  const auto & cp = params_.chainer;
  std::unordered_set<int> visited_set;
  visited_set.insert(seed_idx);

  // owner: 모두 false (독립 chaining)
  std::vector<bool> owner(points.size(), false);

  const int max_extend = cp.max_chain_len - 1;

  // Forward pass (+x 방향)
  auto forward_chain = chain_one_direction(
    points, owner, seed_idx,
    1.0, 0.0, is_left, visited_set,
    max_extend);

  // [seed] + forward → backbone
  std::vector<int> backbone;
  backbone.reserve(1 + forward_chain.size());
  backbone.push_back(seed_idx);
  backbone.insert(backbone.end(), forward_chain.begin(), forward_chain.end());
  return backbone;
}

// ============================================================================
// compute_backtrack_cost — 3-node 윈도우 곡률+거리 비용
// ============================================================================
double YoloLaneClusterNode::compute_backtrack_cost(
  const std::vector<LanePoint> & points,
  const std::vector<int> & backbone,
  int overlap_pos) const
{
  const auto & cp = params_.chainer;

  if (overlap_pos <= 0) return 0.0;

  const int idx_c = backbone[overlap_pos];
  const int idx_b = backbone[overlap_pos - 1];

  const double dx_bc = points[idx_c].x - points[idx_b].x;
  const double dy_bc = points[idx_c].y - points[idx_b].y;
  const double dist_bc = std::sqrt(dx_bc * dx_bc + dy_bc * dy_bc);

  const double cost_dist = dist_bc / cp.d_max;

  // 곡률 비용: 3-node 윈도우가 있을 때만
  double cost_curv = 0.0;
  if (overlap_pos >= 2) {
    const int idx_a = backbone[overlap_pos - 2];
    const double v1x = points[idx_b].x - points[idx_a].x;
    const double v1y = points[idx_b].y - points[idx_a].y;
    const double len_v1 = std::sqrt(v1x * v1x + v1y * v1y);
    const double len_v2 = dist_bc;

    if (len_v1 > 1e-9 && len_v2 > 1e-9) {
      const double dot_val = (v1x * dx_bc + v1y * dy_bc) / (len_v1 * len_v2);
      const double clamped = std::clamp(dot_val, -1.0, 1.0);
      cost_curv = std::acos(clamped);
    }
  }

  return cp.backtrack_w_curv * cost_curv + cp.backtrack_w_dist * cost_dist;
}

// ============================================================================
// rechain_from — truncate 후 재chaining
// ============================================================================
void YoloLaneClusterNode::rechain_from(
  const std::vector<LanePoint> & points,
  std::vector<int> & backbone,
  int rechain_pos,
  bool is_left,
  const std::unordered_set<int> & excluded_set) const
{
  const auto & cp = params_.chainer;

  if (rechain_pos <= 0 || rechain_pos >= static_cast<int>(backbone.size())) return;

  backbone.resize(rechain_pos);

  const int restart_node = backbone[rechain_pos - 1];

  // visited_set: backbone 노드 + excluded 노드
  std::unordered_set<int> visited_set;
  for (int idx : backbone) visited_set.insert(idx);
  for (int idx : excluded_set) visited_set.insert(idx);

  // 진행 방향 복원
  double vx = 1.0, vy = 0.0;
  if (rechain_pos >= 2) {
    const int prev = backbone[rechain_pos - 2];
    const double dx = points[restart_node].x - points[prev].x;
    const double dy = points[restart_node].y - points[prev].y;
    const double d = std::sqrt(dx * dx + dy * dy);
    if (d > 1e-9) {
      vx = dx / d;
      vy = dy / d;
    }
  }

  // owner: 모두 false (독립 재chaining)
  std::vector<bool> owner(points.size(), false);

  const int remaining = cp.max_chain_len - static_cast<int>(backbone.size());
  if (remaining <= 0) return;

  auto new_chain = chain_one_direction(
    points, owner, restart_node,
    vx, vy, is_left, visited_set,
    remaining);

  backbone.insert(backbone.end(), new_chain.begin(), new_chain.end());
}

// ============================================================================
// resolve_overlaps — 좌/우 backbone 중복 해소 backtracking
// ============================================================================
void YoloLaneClusterNode::resolve_overlaps(
  const std::vector<LanePoint> & points,
  std::vector<int> & left_bb,
  std::vector<int> & right_bb) const
{
  const auto & cp = params_.chainer;
  std::unordered_set<int> excluded_set;

  for (int iter = 0; iter < cp.max_backtrack_count; ++iter) {
    // 중복 노드 탐지
    std::unordered_set<int> right_set(right_bb.begin(), right_bb.end());

    int left_overlap_pos = -1;
    int right_overlap_pos = -1;
    int overlap_node = -1;

    for (int i = 0; i < static_cast<int>(left_bb.size()); ++i) {
      if (right_set.count(left_bb[i])) {
        left_overlap_pos = i;
        overlap_node = left_bb[i];
        break;
      }
    }

    if (overlap_node < 0) break;  // 중복 없음

    for (int j = 0; j < static_cast<int>(right_bb.size()); ++j) {
      if (right_bb[j] == overlap_node) {
        right_overlap_pos = j;
        break;
      }
    }

    double left_cost = compute_backtrack_cost(points, left_bb, left_overlap_pos);
    double right_cost = compute_backtrack_cost(points, right_bb, right_overlap_pos);

    excluded_set.insert(overlap_node);

    if (left_cost > right_cost) {
      // left가 부자연스러움 → left rechain
      std::unordered_set<int> excl = excluded_set;
      excl.insert(right_bb.begin(), right_bb.end());
      rechain_from(points, left_bb, left_overlap_pos, true, excl);
    } else if (right_cost > left_cost) {
      // right가 부자연스러움 → right rechain
      std::unordered_set<int> excl = excluded_set;
      excl.insert(left_bb.begin(), left_bb.end());
      rechain_from(points, right_bb, right_overlap_pos, false, excl);
    } else {
      // 동일 → 양쪽 truncate
      if (left_overlap_pos < static_cast<int>(left_bb.size())) {
        left_bb.resize(left_overlap_pos);
      }
      if (right_overlap_pos < static_cast<int>(right_bb.size())) {
        right_bb.resize(right_overlap_pos);
      }
    }
  }
}

// ============================================================================
// backbone_to_boundary — backbone 인덱스 → LaneBoundary 변환
// ============================================================================
ev_msgs::msg::LaneBoundary YoloLaneClusterNode::backbone_to_boundary(
  const std::vector<LanePoint> & points,
  const std::vector<int> & backbone,
  const std_msgs::msg::Header & header) const
{
  ev_msgs::msg::LaneBoundary bd;
  bd.header = header;
  bd.confidence = 1.0f;
  bd.lane_id = 0;

  bd.points.reserve(backbone.size());
  for (int idx : backbone) {
    geometry_msgs::msg::Point pt;
    pt.x = points[idx].x;
    pt.y = points[idx].y;
    pt.z = 0.0;
    bd.points.push_back(pt);
  }

  return bd;
}

}  // namespace yolo_lane_cluster
