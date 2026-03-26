/**
 * @file backbone_extractor.cpp
 * @brief DirectionChainer — Backbone 추출 + 비용함수 + Backtracking 구현
 *
 * [포함 함수]
 *   - extract_backbone():        단방향(forward only) greedy chaining
 *                                [seed] + forward
 *   - chain_one_direction():     단방향 greedy chaining 헬퍼
 *                                (단일 패스: d_max 범위 내 모든 후보를
 *                                 동일 가중치로 탐색, 비용함수 최소 선택)
 *   - compute_cost():            기본 비용함수 w(i,j)
 *   - compute_cost_prime():      확장 비용함수 w'(i,j) = w + λ·C_side
 *   - resolve_overlaps():        독립 체이닝 중복 해소 backtracking 루프
 *   - compute_backtrack_cost():  3-node 윈도우 곡률+거리 비용
 *   - rechain_from():            truncate 후 재체이닝
 *
 * [의존 관계]
 *   - geometry.hpp: dot2(), norm() — 2D 벡터 연산
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

  // C_d: 거리 비용 — 현재 노드(pi) 타입 기준 d_max로 정규화
  const double d_max_cur = (pi.type == PointType::BBOX) ? cp.d_max_bbox : cp.d_max_lane;
  const double C_d = d / d_max_cur;

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
// [단일 패스 탐색 + 클러스터 락]
//   d_max 범위 내 모든 후보(bbox + lane)를 동일 가중치로 탐색.
//   G0(lane_side) + G1(거리) + G2(cone) + G3(lateral) 게이트 적용 후
//   compute_cost_prime()으로 최소 비용 후보 선택.
//
//   클러스터 락: best가 자기 쪽 lane point이면 해당 클러스터(label)만
//   후보로 제한하여 기존 게이트로 클러스터 전체를 chaining.
//   클러스터 후보 소진 시 락 해제 → 정상 탐색 복귀.
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

  // 반대편 lane_side — lane 포인트 중 반대쪽 소속은 chaining 후보에서 제외
  const LaneSide opp_side = is_left ? LaneSide::RIGHT : LaneSide::LEFT;
  const LaneSide my_side  = is_left ? LaneSide::LEFT  : LaneSide::RIGHT;

  // 클러스터 락: 자기 쪽 lane 포인트가 선택되면 해당 클러스터만 후보로 제한
  int32_t locked_cluster_label = -1;

  while (static_cast<int>(chain.size()) < remaining_len) {
    std::vector<int> gated;
    bool had_candidates = false;
    const int n = static_cast<int>(points.size());
    // 현재 노드 타입에 따라 탐색 범위 결정
    const double d_max_cur = (points[current].type == PointType::BBOX)
                              ? cp.d_max_bbox : cp.d_max_lane;
    const double d_max_sq = d_max_cur * d_max_cur;

    // ── 단일 패스: d_max 범위 내 후보 탐색 ──
    // 클러스터 락 중이면 해당 클러스터 lane point만, 아니면 전체(bbox+lane)
    for (int i = 0; i < n; ++i) {
      if (i == current) continue;

      // 클러스터 락: 해당 클러스터의 lane 포인트만 후보
      if (locked_cluster_label >= 0) {
        if (points[i].type != PointType::LANE ||
            points[i].label != locked_cluster_label) continue;
      }

      // G0: lane_side 게이트 — 반대편 차선 포인트 제외 (BBOX는 NONE이라 통과)
      if (points[i].lane_side == opp_side) continue;

      const double dx = points[i].x - points[current].x;
      const double dy = points[i].y - points[current].y;

      // G1: 거리 게이트 — 현재 노드 타입 기준 d_max
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
      Point2D perp = {-v.y, v.x};
      double signed_lat = dx * perp.x + dy * perp.y;
      if (is_left  && signed_lat < -cp.lateral_gate) continue;  // 안쪽(오른쪽) 초과
      if (!is_left && signed_lat >  cp.lateral_gate) continue;  // 안쪽(왼쪽) 초과

      gated.push_back(i);
    }

    if (gated.empty()) {
      // 클러스터 락 중 후보 소진 → 락 해제, 정상 탐색으로 복귀
      if (locked_cluster_label >= 0) {
        locked_cluster_label = -1;
        continue;
      }
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

    // 클러스터 락 진입: 자기 쪽 lane point가 선택되면 해당 클러스터 잠금
    if (locked_cluster_label < 0 &&
        points[best].type == PointType::LANE &&
        points[best].lane_side == my_side) {
      locked_cluster_label = points[best].label;
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
// Backbone 추출 — Forward-only Greedy Chaining
// ============================================================================
// seed에서 forward(+x) 방향으로만 greedy chaining.
// backbone = [seed] + forward.
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

  // Forward pass (+x 방향)
  auto forward_chain = chain_one_direction(
    points, owner, seed_idx,
    {1.0, 0.0}, is_left, visited_set,
    max_extend, stop_reason_forward, cp);

  // [seed] + forward → backbone
  seed_backbone_pos = 0;
  std::vector<int> backbone;
  backbone.reserve(1 + forward_chain.size());
  backbone.push_back(seed_idx);
  backbone.insert(backbone.end(), forward_chain.begin(), forward_chain.end());
  return backbone;
}

// ============================================================================
// compute_backtrack_cost — 3-node 윈도우 곡률+거리 backtracking 비용
// ============================================================================
// backbone에서 overlap_pos 위치의 노드 포함 이전 3개 노드(A, B, C)에 대해:
//   v1 = B - A, v2 = C - B
//   곡률 변화 = acos(dot(v1,v2) / (|v1|·|v2|))
//   거리 = |v2| (B→C 거리, d_max로 정규화)
//   cost = w_curv * 곡률변화 + w_dist * 거리/d_max
//
// 노드가 2개뿐이면 (윈도우 부족) 곡률=0, 거리만 계산.
// 노드가 1개이면 cost=0.
//
double DirectionChainer::compute_backtrack_cost(
  const std::vector<ChainPoint> & points,
  const std::vector<int> & backbone,
  int overlap_pos,
  const PlanningParams::Chainer & cp) const
{
  // overlap_pos가 backbone의 0번째이면 이전 노드 없음 → cost=0
  if (overlap_pos <= 0) return 0.0;

  const int idx_c = backbone[overlap_pos];      // C: 중복 노드
  const int idx_b = backbone[overlap_pos - 1];  // B: 직전 노드

  const double dx_bc = points[idx_c].x - points[idx_b].x;
  const double dy_bc = points[idx_c].y - points[idx_b].y;
  const double dist_bc = std::sqrt(dx_bc * dx_bc + dy_bc * dy_bc);

  // 거리 비용 — B 노드(직전 노드) 타입 기준 d_max로 정규화
  const double d_max_b = (points[idx_b].type == PointType::BBOX)
                          ? cp.d_max_bbox : cp.d_max_lane;
  const double cost_dist = dist_bc / d_max_b;

  // 곡률 비용: 3-node 윈도우가 있을 때만
  double cost_curv = 0.0;
  if (overlap_pos >= 2) {
    const int idx_a = backbone[overlap_pos - 2];  // A: 2칸 이전 노드

    // v1 = B - A
    const double v1x = points[idx_b].x - points[idx_a].x;
    const double v1y = points[idx_b].y - points[idx_a].y;
    const double len_v1 = std::sqrt(v1x * v1x + v1y * v1y);

    // v2 = C - B
    const double len_v2 = dist_bc;

    if (len_v1 > 1e-9 && len_v2 > 1e-9) {
      // dot(v1, v2) / (|v1|·|v2|) = cos(angle)
      const double dot_val = (v1x * dx_bc + v1y * dy_bc) / (len_v1 * len_v2);
      const double clamped = std::clamp(dot_val, -1.0, 1.0);
      cost_curv = std::acos(clamped);  // [0, π] 범위의 각도 변화
    }
  }

  return cp.backtrack_w_curv * cost_curv + cp.backtrack_w_dist * cost_dist;
}

// ============================================================================
// rechain_from — backbone의 특정 위치에서 truncate 후 재체이닝
// ============================================================================
// backbone[rechain_pos] 이후를 삭제하고, backbone[rechain_pos-1] 노드에서
// forward 방향으로 chain_one_direction을 다시 실행한다.
// excluded_set에 포함된 노드는 visited_set에 미리 삽입하여 영구 제외.
//
void DirectionChainer::rechain_from(
  const std::vector<ChainPoint> & points,
  std::vector<int> & backbone,
  int rechain_pos,
  bool is_left,
  const std::unordered_set<int> & excluded_set,
  const PlanningParams::Chainer & cp) const
{
  if (rechain_pos <= 0 || rechain_pos >= static_cast<int>(backbone.size())) return;

  // rechain_pos 이후 삭제
  backbone.resize(rechain_pos);

  // 재체이닝 시작 노드 = truncate 직전 노드
  const int restart_node = backbone[rechain_pos - 1];

  // visited_set 재구성: backbone에 이미 있는 노드 + excluded 노드
  std::unordered_set<int> visited_set;
  for (int idx : backbone) {
    visited_set.insert(idx);
  }
  for (int idx : excluded_set) {
    visited_set.insert(idx);
  }

  // 진행 방향 복원: 마지막 2개 노드로 방향 벡터 계산
  Point2D dir = {1.0, 0.0};  // 기본: forward
  if (rechain_pos >= 2) {
    const int prev = backbone[rechain_pos - 2];
    const double dx = points[restart_node].x - points[prev].x;
    const double dy = points[restart_node].y - points[prev].y;
    const double d = std::sqrt(dx * dx + dy * dy);
    if (d > 1e-9) {
      dir = {dx / d, dy / d};
    }
  }

  // owner = all NONE (독립 재체이닝이므로 상대 chain 고려 안 함)
  std::vector<NodeOwner> owner_none(points.size(), NodeOwner::NONE);

  const int remaining = cp.max_chain_len - static_cast<int>(backbone.size());
  if (remaining <= 0) return;

  StopReason stop_reason = StopReason::MAX_LEN;
  auto new_chain = chain_one_direction(
    points, owner_none, restart_node, dir, is_left,
    visited_set, remaining, stop_reason, cp);

  // 재체이닝 결과 append
  backbone.insert(backbone.end(), new_chain.begin(), new_chain.end());
}

// ============================================================================
// resolve_overlaps — 독립 체이닝 중복 해소 backtracking 루프
// ============================================================================
// [알고리즘]
//   1) left_bb와 right_bb에서 공통 노드(중복) 탐지
//   2) 각 backbone 순서상 가장 빨리 등장하는 중복 노드 선택
//   3) 양쪽 3-node 윈도우 비용 비교:
//      - 한쪽이 높으면: 그 쪽에서 truncate + 중복 노드 제외 후 재체이닝
//      - 동일하면: 양쪽 모두 중복 노드 이후 truncate (재체이닝 없음)
//   4) max_backtrack_count까지 반복
//
void DirectionChainer::resolve_overlaps(
  const std::vector<ChainPoint> & points,
  std::vector<int> & left_bb,
  std::vector<int> & right_bb,
  const PlanningParams::Chainer & cp) const
{
  std::unordered_set<int> excluded_set;  // 영구 제외 노드 (누적)

  // 시드 위치 로그 (backbone 첫 번째 노드 = seed)
  if (!left_bb.empty() && !right_bb.empty()) {
    int ls = left_bb.front(), rs = right_bb.front();
    std::fprintf(stderr,
      "[backtrack] === START === left_bb:%zu pts, right_bb:%zu pts\n"
      "[backtrack] left_seed  node=%d (%.2f, %.2f)\n"
      "[backtrack] right_seed node=%d (%.2f, %.2f)\n",
      left_bb.size(), right_bb.size(),
      ls, points[ls].x, points[ls].y,
      rs, points[rs].x, points[rs].y);
  }

  for (int iter = 0; iter < cp.max_backtrack_count; ++iter) {
    // 중복 노드 탐지: right_bb를 set으로 만들어 left_bb에서 검색
    std::unordered_set<int> right_set(right_bb.begin(), right_bb.end());

    // 양쪽에서 가장 먼저 등장하는 중복 노드와 그 위치 탐색
    int left_overlap_pos = -1;
    int right_overlap_pos = -1;
    int overlap_node = -1;

    // left_bb에서 가장 먼저 등장하는 중복 노드
    for (int i = 0; i < static_cast<int>(left_bb.size()); ++i) {
      if (right_set.count(left_bb[i])) {
        left_overlap_pos = i;
        overlap_node = left_bb[i];
        break;
      }
    }

    if (overlap_node < 0) {
      std::fprintf(stderr, "[backtrack] iter %d — no overlap, done\n", iter);
      break;
    }

    // right_bb에서 해당 노드의 위치 탐색
    for (int j = 0; j < static_cast<int>(right_bb.size()); ++j) {
      if (right_bb[j] == overlap_node) {
        right_overlap_pos = j;
        break;
      }
    }

    // 3-node 윈도우 비용 계산
    double left_cost = compute_backtrack_cost(
      points, left_bb, left_overlap_pos, cp);
    double right_cost = compute_backtrack_cost(
      points, right_bb, right_overlap_pos, cp);

    const auto & op = points[overlap_node];
    int left_bb_len_before = static_cast<int>(left_bb.size());
    int right_bb_len_before = static_cast<int>(right_bb.size());

    std::fprintf(stderr,
      "[backtrack] iter %d — overlap node=%d (%.2f, %.2f) type=%s\n"
      "  left_pos=%d/%d cost=%.3f | right_pos=%d/%d cost=%.3f\n",
      iter, overlap_node, op.x, op.y,
      (op.type == PointType::BBOX ? "BBOX" : "LANE"),
      left_overlap_pos, left_bb_len_before, left_cost,
      right_overlap_pos, right_bb_len_before, right_cost);

    // 중복 노드를 영구 제외 리스트에 추가
    excluded_set.insert(overlap_node);

    if (left_cost > right_cost) {
      // left가 부자연스러움 → left에서 backtracking
      // 상대(right) backbone 노드를 excluded에 포함 → 재체이닝 시 상대 영역 진입 방지
      std::unordered_set<int> excl_with_opponent = excluded_set;
      excl_with_opponent.insert(right_bb.begin(), right_bb.end());
      std::fprintf(stderr, "  → LEFT loses (cost %.3f > %.3f), rechain left from pos %d (excl +%zu right nodes)\n",
        left_cost, right_cost, left_overlap_pos, right_bb.size());
      rechain_from(points, left_bb, left_overlap_pos, true, excl_with_opponent, cp);
      std::fprintf(stderr, "  → left_bb: %d → %zu pts\n",
        left_bb_len_before, left_bb.size());
    } else if (right_cost > left_cost) {
      // right가 부자연스러움 → right에서 backtracking
      // 상대(left) backbone 노드를 excluded에 포함 → 재체이닝 시 상대 영역 진입 방지
      std::unordered_set<int> excl_with_opponent = excluded_set;
      excl_with_opponent.insert(left_bb.begin(), left_bb.end());
      std::fprintf(stderr, "  → RIGHT loses (cost %.3f > %.3f), rechain right from pos %d (excl +%zu left nodes)\n",
        right_cost, left_cost, right_overlap_pos, left_bb.size());
      rechain_from(points, right_bb, right_overlap_pos, false, excl_with_opponent, cp);
      std::fprintf(stderr, "  → right_bb: %d → %zu pts\n",
        right_bb_len_before, right_bb.size());
    } else {
      // 비용 동일 → 양쪽 모두 중복 노드 이후 truncate (재체이닝 없음)
      std::fprintf(stderr, "  → TIE (cost %.3f), both truncate\n", left_cost);
      if (left_overlap_pos < static_cast<int>(left_bb.size())) {
        left_bb.resize(left_overlap_pos);
      }
      if (right_overlap_pos < static_cast<int>(right_bb.size())) {
        right_bb.resize(right_overlap_pos);
      }
      std::fprintf(stderr, "  → left_bb: %d → %zu, right_bb: %d → %zu\n",
        left_bb_len_before, left_bb.size(),
        right_bb_len_before, right_bb.size());
    }
  }

  // 총 중복 잔존 확인
  if (!left_bb.empty() && !right_bb.empty()) {
    std::unordered_set<int> right_set(right_bb.begin(), right_bb.end());
    int remaining_overlaps = 0;
    for (int idx : left_bb) {
      if (right_set.count(idx)) ++remaining_overlaps;
    }
    std::fprintf(stderr,
      "[backtrack] === END === left_bb:%zu right_bb:%zu remaining_overlaps=%d\n",
      left_bb.size(), right_bb.size(), remaining_overlaps);
  }
}

}  // namespace chaining_costmap_ver
