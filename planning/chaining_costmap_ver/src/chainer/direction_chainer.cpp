/**
 * @file direction_chainer.cpp
 * @brief DirectionChainer — Owner-Label 기반 순차 체이닝 구현부 (v3)
 *
 * [파일 구조]
 *   이 파일은 direction_chainer.hpp에 선언된 DirectionChainer 클래스의
 *   모든 멤버 함수를 구현한다. 각 함수는 파이프라인의 한 단계에 대응한다.
 *
 * [파이프라인 요약]
 *   준비: find_seed()           — 좌/우 시작점 선택 (최근접점 기반)
 *         build_graph()         — kNN + G1,G3 게이트로 undirected 그래프 구성
 *   1단계: extract_backbone()   — left backbone 확정 (owner==NONE 후보)
 *   2단계: extract_backbone()   — right backbone 확정 (LEFT_BACKBONE 제외)
 *   3단계: extract_branches()   — left branch 확정 (backbone 순회 BFS)
 *   4단계: extract_branches()   — right branch 확정 (LEFT_BACKBONE+LEFT_BRANCH 제외)
 *   5단계: resample_component() — 좌/우 각각 전 edge를 일정 간격으로 보간
 *
 * [의존 관계]
 *   - geometry.hpp: dot2() — 2D 벡터 내적 함수
 *   - types.hpp: ChainPoint, ChainingGraph, BranchInfo, StopReason, NodeOwner 등
 *   - params.hpp: PlanningParams::Chainer — 모든 튜닝 파라미터
 */
#include "chaining_costmap_ver/chainer/direction_chainer.hpp"
#include "chaining_costmap_ver/common/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <numeric>

namespace chaining_costmap_ver
{

// ============================================================================
// 메인 chain() — Owner-Label 기반 순차 체이닝
// ============================================================================
//
// [전체 흐름 다이어그램]
//
//   입력: ChainPoint[] (콘+차선 혼합)
//     │
//     ▼
//   준비: find_seed() × 2 → 좌측/우측 seed 선택
//         build_graph() → kNN+게이트 기반 undirected 그래프
//         owner[] 초기화 (all NONE)
//     │
//     ▼
//   1단계: extract_backbone(left) → LEFT_BACKBONE 라벨 부여
//     │
//     ▼
//   2단계: extract_backbone(right) → RIGHT_BACKBONE 라벨 부여
//     │                              (LEFT_BACKBONE 자동 제외)
//     ▼
//   3단계: extract_branches(left) → LEFT_BRANCH 라벨 부여
//     │                             (backbone 제외, LEFT_BRANCH 중복 허용)
//     ▼
//   4단계: extract_branches(right) → RIGHT_BRANCH 라벨 부여
//     │                              (LEFT_BACKBONE+LEFT_BRANCH 제외)
//     ▼
//   5단계: resample_component() × 2 → 균등 보간
//     │
//     ▼
//   출력: DirectionChainResult (left + right SideResult)
//
// ============================================================================

DirectionChainResult DirectionChainer::chain(
  const std::vector<ChainPoint> & points,
  const PlanningParams & params) const
{
  DirectionChainResult result;
  const auto & cp = params.chainer;

  // ── 준비: 입력 검사 ──
  const auto & filtered = points;
  if (filtered.size() < 2) return result;

  // ── 준비: Seed 선택 ──
  int left_seed = find_seed(filtered, true, cp);
  int right_seed = find_seed(filtered, false, cp);
  if (left_seed < 0 && right_seed < 0) return result;

  // ── 준비: Undirected Graph 구성 ──
  auto graph = build_graph(filtered, cp);

  // ── 준비: Owner 배열 초기화 ──
  // 모든 노드를 NONE으로 시작. 각 단계에서 라벨을 부여한다.
  std::vector<NodeOwner> owner(filtered.size(), NodeOwner::NONE);

  // ── 1단계: Left Backbone 확정 ──
  std::vector<int> left_backbone_ids;
  StopReason left_stop = StopReason::NO_CANDIDATE;
  if (left_seed >= 0) {
    left_backbone_ids = extract_backbone(
      filtered, owner, left_seed, true, left_stop, cp);
    for (int idx : left_backbone_ids) {
      owner[idx] = NodeOwner::LEFT_BACKBONE;
    }
  }

  // ── 2단계: Right Backbone 확정 ──
  // LEFT_BACKBONE 노드는 owner!=NONE이라 자동 제외된다.
  std::vector<int> right_backbone_ids;
  StopReason right_stop = StopReason::NO_CANDIDATE;
  if (right_seed >= 0) {
    right_backbone_ids = extract_backbone(
      filtered, owner, right_seed, false, right_stop, cp);
    for (int idx : right_backbone_ids) {
      owner[idx] = NodeOwner::RIGHT_BACKBONE;
    }
  }

  // ── 3단계: Left Branch 확정 ──
  // 허용: NONE, LEFT_BRANCH / 차단: LEFT_BACKBONE, RIGHT_BACKBONE
  std::vector<BranchInfo> left_branches;
  if (!left_backbone_ids.empty()) {
    left_branches = extract_branches(
      graph, filtered, left_backbone_ids, owner,
      NodeOwner::LEFT_BRANCH, cp);
  }

  // ── 4단계: Right Branch 확정 ──
  // 허용: NONE, RIGHT_BRANCH / 차단: LEFT_BACKBONE, RIGHT_BACKBONE, LEFT_BRANCH
  std::vector<BranchInfo> right_branches;
  if (!right_backbone_ids.empty()) {
    right_branches = extract_branches(
      graph, filtered, right_backbone_ids, owner,
      NodeOwner::RIGHT_BRANCH, cp);
  }

  // ── 5단계: 좌/우 각각 Resample ──
  // Left side SideResult 구성
  if (!left_backbone_ids.empty()) {
    result.left.seed_idx = left_seed;
    result.left.goal_idx = left_backbone_ids.back();
    result.left.stop_reason = left_stop;
    result.left.backbone.reserve(left_backbone_ids.size());
    for (int idx : left_backbone_ids) {
      result.left.backbone.push_back(filtered[idx]);
    }
    result.left.branches = std::move(left_branches);
    result.left.component = resample_component(
      filtered, left_backbone_ids, result.left.branches, cp.resample_ds);
  }

  // Right side SideResult 구성
  if (!right_backbone_ids.empty()) {
    result.right.seed_idx = right_seed;
    result.right.goal_idx = right_backbone_ids.back();
    result.right.stop_reason = right_stop;
    result.right.backbone.reserve(right_backbone_ids.size());
    for (int idx : right_backbone_ids) {
      result.right.backbone.push_back(filtered[idx]);
    }
    result.right.branches = std::move(right_branches);
    result.right.component = resample_component(
      filtered, right_backbone_ids, result.right.branches, cp.resample_ds);
  }

  // ── unchained 포인트 수집 ──
  // owner가 NONE인 노드 = 어떤 체인에도 속하지 못한 점
  // → costmap에서 CONE 비용으로 보수적 처리 (미확인 장애물)
  for (size_t i = 0; i < filtered.size(); ++i) {
    if (owner[i] == NodeOwner::NONE) {
      result.unchained.push_back(filtered[i]);
    }
  }

  // 최소 한쪽 backbone이 생성되었으면 유효한 결과
  result.valid = (!result.left.backbone.empty() ||
                  !result.right.backbone.empty());
  return result;
}

// ============================================================================
// 1단계: Seed 선택 (Seed Selection)
// ============================================================================
//
// [목적]
//   체이닝(greedy chaining)의 시작점을 결정한다.
//   좋은 seed를 선택하면 backbone이 올바른 방향으로 뻗어나간다.
//
// [seed 선택 전략]
//
//   1) 전방 필터: x ≥ -2.0 인 점만 후보
//      → 후방 2m 이상 점에서 시작하면 체인이 뒤로 뻗어 무의미
//
//   2) side_seed_y 가드:
//      ┌────────────────────────────┐
//      │         y > side_seed_y    │  ← 좌측 seed 영역
//      │ · · · · · · · · · · · · · │
//      │ ─ ─ ─ ─ ─ y=0 ─ ─ ─ ─ ─ │  ← 중심선 부근 (양쪽 다 제외)
//      │ · · · · · · · · · · · · · │
//      │        y < -side_seed_y   │  ← 우측 seed 영역
//      └────────────────────────────┘
//      → 중심선 근처 점은 좌/우 구분이 애매하므로 seed 후보에서 제외
//
//   3) ego 최근접: 후보 중 원점(ego)에서 거리가 가장 가까운 점 선택
//      → 가까운 점일수록 좌/우 구분이 확실함
//      → 콘/차선 구분 없이 가장 가까운 점을 seed로 사용
//
// [반환값]
//   seed 인덱스 (후보 없으면 -1)
//
// ============================================================================

int DirectionChainer::find_seed(
  const std::vector<ChainPoint> & points,
  bool is_left,
  const PlanningParams::Chainer & cp) const
{
  int best = -1;           // 전체 최근접 후보 인덱스
  double best_dist_sq = std::numeric_limits<double>::max();       // 최소 거리² (전체)
  const int n = static_cast<int>(points.size());

  for (int i = 0; i < n; ++i) {
    // x < -2.0 (후방 2m 이상)인 점은 seed 후보에서 제외
    if (points[i].x < -2.0) continue;

    // side_seed_y 가드: 중심선 부근 점 제외
    // 좌측: y가 side_seed_y 미만이면 중심선에 너무 가까움 → 제외
    // 우측: y가 -side_seed_y 초과이면 중심선에 너무 가까움 → 제외
    if (is_left) {
      if (points[i].y < cp.side_seed_y) continue;
    } else {
      if (points[i].y > -cp.side_seed_y) continue;
    }

    // ego(원점)에서의 거리 제곱 (sqrt 생략하여 비교 효율화)
    const double d_sq = points[i].x * points[i].x +
                        points[i].y * points[i].y;

    // 전체 최근접 갱신
    if (d_sq < best_dist_sq) {
      best_dist_sq = d_sq;
      best = i;
    }
  }  // for

  // 전체 최근접 점을 seed로 사용 (차선점 포함)
  return best;
}

// ============================================================================
// 2단계: Undirected Graph 구성 (Build Graph)
// ============================================================================
//
// [목적]
//   모든 경계점 사이의 "연결 가능성"을 그래프로 표현한다.
//   이 그래프는 3단계 BFS(component 추출)와 5단계 BFS(branch 추출)에서 사용된다.
//
// [왜 kNN을 사용하는가?]
//   - 점 N개에 대해 모든 쌍을 검사하면 O(N²) 번 연산이 필요하다.
//   - kNN으로 각 점의 k개 이웃만 검사하면 O(N·k) 번으로 줄어든다.
//   - 경계점은 물리적으로 "가까운 점끼리 연결"되므로 kNN이 자연스럽다.
//
// [게이트 필터]
//   kNN이 반환한 후보에 추가 필터(게이트)를 적용한다:
//
//   G1 (거리 게이트): d(i,j) ≤ d_max
//     → 거리가 d_max보다 먼 점은 같은 경계가 아닐 확률이 높다.
//     → kNN은 상대적 순서만 보므로, 절대 거리 제한이 별도로 필요하다.
//
//   G3 (횡오차 게이트): |Δy| ≤ lateral_gate
//     → y 좌표 차이가 큰 점끼리의 연결을 차단한다.
//     → 좌측 경계 점과 우측 경계 점이 그래프에서 직접 연결되는 것을 방지.
//     → 2단계에서는 진행 방향 v가 없으므로 단순 Δy(y 좌표 차이)를 사용한다.
//        (4단계 backbone에서는 v 수직 방향 투영을 사용하는 더 정밀한 방식)
//
//   ※ G2 (전방 콘 게이트)는 여기서 적용하지 않는다.
//     → 이유: undirected 그래프에는 "진행 방향" 개념이 없다.
//     → G2는 4단계 backbone 추출에서 동적으로 적용된다.
//
// [양방향 edge]
//   undirected 그래프이므로, i→j edge를 추가하면 j→i도 함께 추가한다.
//   중복 체크로 같은 edge가 두 번 들어가는 것을 방지한다.
//
// ============================================================================

ChainingGraph DirectionChainer::build_graph(
  const std::vector<ChainPoint> & points,
  const PlanningParams::Chainer & cp) const
{
  const int n = static_cast<int>(points.size());
  ChainingGraph graph;
  graph.undirected.resize(n);  // N개 노드의 인접 리스트 초기화

  for (int i = 0; i < n; ++i) {
    // i번째 점의 k-최근접 이웃 탐색 (brute-force)
    auto neighbors = knn(points, i, cp.k);

    for (int j : neighbors) {
      if (j == i) continue;  // 자기 자신은 제외

      // ── G1: 거리 게이트 ──
      // 두 점 사이의 유클리드 거리가 d_max를 초과하면 연결하지 않음
      const double dx = points[j].x - points[i].x;
      const double dy = points[j].y - points[i].y;
      const double d = std::sqrt(dx * dx + dy * dy);
      if (d > cp.d_max) continue;

      // ── G3: 횡오차 게이트 (undirected 버전) ──
      // y 좌표 차이의 절대값이 lateral_gate를 초과하면 연결하지 않음
      // → 좌측 경계와 우측 경계 점이 직접 연결되는 것을 방지
      if (std::abs(dy) > cp.lateral_gate) continue;

      // ── 양방향 edge 추가 ──
      // 이미 연결되어 있는지 중복 체크 (선형 탐색: 인접 리스트가 작으므로 OK)
      auto & adj_i = graph.undirected[i];
      if (std::find(adj_i.begin(), adj_i.end(), j) == adj_i.end()) {
        adj_i.push_back(j);           // i → j
        graph.undirected[j].push_back(i);  // j → i (양방향)
      }
    }
  }

  return graph;
}

// ============================================================================
// kNN — brute-force k-최근접 이웃 탐색
// ============================================================================
//
// [목적]
//   주어진 점(query)에서 가장 가까운 k개의 이웃 점을 찾는다.
//   2단계(그래프 구성)와 4단계(backbone 추출)에서 호출된다.
//
// [왜 brute-force인가?]
//   - KD-Tree, Ball-Tree 등의 공간 인덱스를 쓰면 O(n·log n) 전처리 후
//     O(log n) 탐색이 가능하지만, 경계점 수가 수십~수백 개로 적다.
//   - brute-force O(n)도 실시간 처리에 충분하다 (μs 단위).
//   - 구현이 단순하여 디버깅/유지보수가 쉽다.
//
// [최적화: partial_sort]
//   - 전체 정렬 O(n·log n) 대신 partial_sort O(n·log k)를 사용한다.
//   - 상위 k개만 정렬하면 되므로 k << n일 때 효율적이다.
//
// [거리 제곱 사용]
//   - sqrt를 생략하고 거리 제곱(d²)으로 비교한다.
//   - d² 순서와 d 순서는 동일하므로 결과에 영향 없음.
//   - sqrt 연산 N번을 절약하여 미세한 성능 이득.
//
// ============================================================================

std::vector<int> DirectionChainer::knn(
  const std::vector<ChainPoint> & points,
  int query_idx,
  int k) const
{
  const int n = static_cast<int>(points.size());
  const double qx = points[query_idx].x;  // 쿼리 점의 x 좌표
  const double qy = points[query_idx].y;  // 쿼리 점의 y 좌표

  // (거리², 인덱스) 쌍을 수집 — 자기 자신 제외
  std::vector<std::pair<double, int>> dists;
  dists.reserve(n);
  for (int i = 0; i < n; ++i) {
    if (i == query_idx) continue;  // 자기 자신은 이웃에서 제외
    const double dx = points[i].x - qx;
    const double dy = points[i].y - qy;
    dists.push_back({dx * dx + dy * dy, i});  // 거리² (sqrt 생략)
  }

  // 상위 k개만 partial sort (전체 정렬보다 효율적)
  // actual_k: 점의 수가 k보다 적을 수 있으므로 min 처리
  const int actual_k = std::min(k, static_cast<int>(dists.size()));
  std::partial_sort(dists.begin(), dists.begin() + actual_k, dists.end());

  // k개 이웃의 인덱스만 추출하여 반환 (거리 오름차순)
  std::vector<int> result;
  result.reserve(actual_k);
  for (int i = 0; i < actual_k; ++i) {
    result.push_back(dists[i].second);
  }
  return result;
}

// ============================================================================
// 비용함수 w(i, j) — 기본 비용 계산
// ============================================================================
//
// [목적]
//   현재 점 pi에서 후보 다음 점 pj로의 "연결 비용"을 계산한다.
//   비용이 낮을수록 더 자연스러운 연결이다.
//
// [비용함수 수식]
//   w(i,j) = α·C_d + β·C_a + γ·C_lat + δ·C_size
//
// [각 비용 항의 물리적 의미]
//
//   ┌──────────────────────────────────────────────────────────┐
//   │ C_d (거리 비용) = d(i,j) / d_max                        │
//   │                                                          │
//   │   물리적 의미: 가까운 점을 선호한다.                       │
//   │   경계 위의 점들은 일정 간격으로 존재하므로,               │
//   │   너무 먼 점으로 점프하면 중간에 빠진 점이 있다는 뜻이다.  │
//   │   0~1 범위로 정규화 (d=0이면 0, d=d_max이면 1).          │
//   │   가중치 α가 클수록 가까운 점을 강하게 선호한다.           │
//   └──────────────────────────────────────────────────────────┘
//
//   ┌──────────────────────────────────────────────────────────┐
//   │ C_a (방향 오차 비용) = angle(v_i, u_ij) / θ_max         │
//   │                                                          │
//   │   v_i: 현재 진행 방향 단위벡터 (이전→현재 방향)           │
//   │   u_ij: 현재→후보 방향 단위벡터                           │
//   │   angle: 두 벡터 사이의 각도 (0~π)                       │
//   │   θ_max: forward_cone_deg/2 (라디안 변환)                │
//   │                                                          │
//   │   물리적 의미: 현재 진행 방향과 일치하는 점을 선호한다.     │
//   │   → 급격한 방향 전환(지그재그) 방지                       │
//   │   → 경계선이 부드럽게 이어지도록 유도                     │
//   │   가중치 β가 클수록 방향 일관성을 강하게 요구한다.         │
//   └──────────────────────────────────────────────────────────┘
//
//   ┌──────────────────────────────────────────────────────────┐
//   │ C_lat (횡오차 비용) = |lat_offset| / lateral_gate        │
//   │                                                          │
//   │   lat_offset: v_i의 수직 방향(perp)으로의 투영 거리       │
//   │   perp = (-v_i.y, v_i.x) — 진행 방향의 왼쪽 수직벡터     │
//   │                                                          │
//   │   물리적 의미: 진행 방향의 "옆쪽"으로 벗어난 정도.         │
//   │   → 체인이 옆으로 튀는 것을 방지한다.                     │
//   │   → C_a는 각도만 보지만, C_lat는 실제 횡방향 거리를 본다. │
//   │   가중치 γ가 클수록 일직선 연결을 선호한다.                │
//   └──────────────────────────────────────────────────────────┘
//
//   ┌──────────────────────────────────────────────────────────┐
//   │ C_size (크기 변화 비용) — 콘↔콘 전용                     │
//   │   = |size_j - size_i| / (size_i + ε)                     │
//   │                                                          │
//   │   size = AABB의 X+Y 합 (콘의 겉보기 크기 근사치)         │
//   │                                                          │
//   │   물리적 의미: 비슷한 크기의 콘끼리 연결을 선호한다.       │
//   │   같은 종류의 콘(PE 드럼 500mm)은 크기가 유사하다.        │
//   │   크기가 갑자기 바뀌면 다른 종류의 장애물일 수 있다.       │
//   │   → 콘↔차선 / 차선↔차선일 때는 C_size = 0 (적용 안 함)   │
//   │   가중치 δ가 클수록 크기 일관성을 강하게 요구한다.         │
//   └──────────────────────────────────────────────────────────┘
//
// ============================================================================

double DirectionChainer::compute_cost(
  const ChainPoint & pi,
  const ChainPoint & pj,
  const Point2D & v_i,
  const PlanningParams::Chainer & cp) const
{
  // pi→pj 변위 벡터와 거리 계산
  const double dx = pj.x - pi.x;
  const double dy = pj.y - pi.y;
  const double d = std::sqrt(dx * dx + dy * dy);

  // ── C_d: 거리 비용 ──
  // d_max로 정규화하여 0~1 범위로 매핑 (d=0이면 0, d=d_max이면 1)
  const double C_d = d / cp.d_max;

  // ── C_a: 방향 오차 비용 ──
  // 현재 진행 방향 v_i와 pi→pj 방향 u_ij 사이의 각도를 계산
  double C_a = 0.0;
  if (d > 1e-9) {  // 거리가 0에 가까우면 방향 계산 불가 → C_a = 0
    // u_ij: pi에서 pj로의 단위 방향벡터
    Point2D u_ij = {dx / d, dy / d};
    // 내적으로 cos(angle) 계산 → acos로 각도 변환
    double cos_angle = dot2(v_i, u_ij);
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);  // 부동소수점 오차 방지
    double angle = std::acos(cos_angle);
    // θ_max: forward_cone_deg의 반각(half angle)을 라디안으로 변환
    // 예: forward_cone_deg=120° → θ_max = 60° = π/3 rad
    const double theta_max_rad = cp.forward_cone_deg * M_PI / 360.0;
    C_a = angle / theta_max_rad;  // 정규화 (0° → 0, θ_max → 1)
  }

  // ── C_lat: 횡오차 비용 ──
  // v_i의 수직(왼쪽) 방향으로 pi→pj 벡터를 투영한 크기
  // = 진행 방향에 대한 "옆으로 벗어난 거리"
  Point2D perp = {-v_i.y, v_i.x};  // v_i를 90° 회전 (왼쪽 수직)
  double lat_offset = std::abs(dx * perp.x + dy * perp.y);  // 내적 = 투영 길이
  const double C_lat = lat_offset / cp.lateral_gate;  // 정규화

  // ── C_size: 크기 변화 비용 (콘↔콘 전용) ──
  // 양쪽 모두 콘일 때만 계산. 차선점은 AABB 크기 정보가 없으므로 0.
  double C_size = 0.0;
  if (pi.type == PointType::CONE && pj.type == PointType::CONE) {
    // size = AABB의 X+Y 합 (콘의 겉보기 크기 근사)
    double size_i = pi.size_x + pi.size_y;
    double size_j = pj.size_x + pj.size_y;
    const double eps = 1e-6;  // 0 나누기 방지
    // 상대적 크기 변화율: |크기차이| / 기준크기
    C_size = std::abs(size_j - size_i) / (size_i + eps);
  }

  // ── 가중합 ──
  // 각 비용 항에 가중치를 곱해서 합산
  // 기본값: α=1.0, β=1.2, γ=0.6, δ=0.2
  // β가 가장 커서 방향 일관성이 가장 중요하다 (지그재그 방지)
  return cp.alpha * C_d + cp.beta * C_a + cp.gamma * C_lat + cp.delta * C_size;
}

// ============================================================================
// w' = w + λ_side · C_side — 확장 비용함수 (side preference 포함)
// ============================================================================
//
// [목적]
//   기본 비용 w에 "측면 선호도 페널티" C_side를 추가한다.
//   체인이 반대편 경계(중심선) 쪽으로 이동하는 것을 방지한다.
//
// [C_side의 물리적 의미]
//
//   ┌─────── 좌측 경계 (is_left = true) ──────┐
//   │                                          │
//   │   y ↑  좌측 경계 방향 (바깥쪽)            │
//   │   │  ···· pi ────→ pj (y 유지: OK)       │
//   │   │  ···· pi ──↘ pj  (y 감소: 안쪽!)     │
//   │   │               → C_side = (pi.y-pj.y) │
//   │   ─────── y=0 (중심선) ─────────          │
//   │                                          │
//   │   C_side = max(0, pi.y - pj.y)           │
//   │   → y가 감소(안쪽으로 이동) 시 페널티 부과 │
//   │   → y가 증가(바깥쪽) 또는 유지 시 페널티 0 │
//   └──────────────────────────────────────────┘
//
//   ┌─────── 우측 경계 (is_left = false) ─────┐
//   │                                          │
//   │   ─────── y=0 (중심선) ─────────          │
//   │   │               → C_side = (pj.y-pi.y) │
//   │   │  ···· pi ──↗ pj  (y 증가: 안쪽!)     │
//   │   │  ···· pi ────→ pj (y 유지: OK)       │
//   │   y ↓  우측 경계 방향 (바깥쪽)            │
//   │                                          │
//   │   C_side = max(0, pj.y - pi.y)           │
//   │   → y가 증가(안쪽으로 이동) 시 페널티 부과 │
//   │   → y가 감소(바깥쪽) 또는 유지 시 페널티 0 │
//   └──────────────────────────────────────────┘
//
// [λ_side 튜닝 가이드]
//   - λ_side = 0: C_side 무시 (좌/우 분리 약함)
//   - λ_side = 0.5 (기본값): 적당한 좌/우 분리
//   - λ_side > 1.0: 강한 좌/우 분리 (중심선 방향 이동 거의 차단)
//
// ============================================================================

double DirectionChainer::compute_cost_prime(
  const ChainPoint & pi,
  const ChainPoint & pj,
  const Point2D & v_i,
  bool is_left,
  const PlanningParams::Chainer & cp) const
{
  // 기본 비용 w(i,j) 계산 (C_d + C_a + C_lat + C_size)
  double w = compute_cost(pi, pj, v_i, cp);

  // ── C_side: 측면 선호도 페널티 ──
  double C_side = 0.0;
  if (is_left) {
    // 좌측 경계: y가 감소(=중심선 쪽으로 이동)하면 페널티
    // max(0, ...)로 바깥쪽 이동은 페널티 없음
    C_side = std::max(0.0, pi.y - pj.y) / cp.lateral_gate;
  } else {
    // 우측 경계: y가 증가(=중심선 쪽으로 이동)하면 페널티
    C_side = std::max(0.0, pj.y - pi.y) / cp.lateral_gate;
  }

  // w' = w + λ_side · C_side
  return w + cp.lambda_side * C_side;
}

// ============================================================================
// 1-2단계: Backbone 추출 (Greedy Chaining)
// ============================================================================
//
// [목적]
//   seed부터 전방으로 이어지는 "주 경계선"(backbone)을 추출한다.
//   이 backbone이 한쪽 경계의 핵심 구조이다.
//
// [Owner 기반 필터링]
//   기존 component_ids 대신 owner 배열을 사용한다.
//   owner[j] == NONE인 노드만 후보로 허용하므로:
//   - 1단계(left backbone): 모든 NONE 노드가 후보
//   - 2단계(right backbone): LEFT_BACKBONE 노드는 자동 제외
//
// [Greedy Chaining 알고리즘]
//
//   seed ──→ ● ──→ ● ──→ ● ──→ ● ──→ goal
//            ↑
//         매 스텝마다:
//         1) 현재 노드의 kNN 후보 탐색
//         2) owner==NONE 필터링 + 3개 게이트(G1+G2+G3) 적용
//         3) 통과한 후보들의 w' 비용 계산
//         4) w' 최소인 노드로 이동, 진행 방향 갱신
//         5) 반복 (종료 조건 충족 시 중단)
//
// [3개 게이트 — backbone 전용]
//
//   G1 (거리 게이트): d(cur, j) ≤ d_max  AND  d > 0
//   G2 (전방 콘 게이트): angle(v, u_ij) ≤ cone_half_rad
//   G3 (횡오차 게이트): |lat_proj| ≤ lateral_gate
//
// [종료 조건과 StopReason]
//   - MAX_LEN: max_chain_len 도달 (정상 종료)
//   - NO_CANDIDATE: kNN 중 NONE 후보가 없음 (경계 끝)
//   - ALL_GATED: 후보는 있지만 게이트를 모두 탈락
//
// ============================================================================

std::vector<int> DirectionChainer::extract_backbone(
  const std::vector<ChainPoint> & points,
  const std::vector<NodeOwner> & owner,
  int seed_idx,
  bool is_left,
  StopReason & stop_reason,
  const PlanningParams::Chainer & cp) const
{

  // backbone: seed에서 시작하는 노드 인덱스 리스트
  std::vector<int> backbone;
  backbone.push_back(seed_idx);

  // backbone 내 중복 방문 방지용 set
  std::unordered_set<int> visited_set;
  visited_set.insert(seed_idx);

  int current = seed_idx;

  // 초기 진행 방향: base_link의 전방 (x축 양의 방향)
  // 체이닝이 진행될수록 실제 이동 방향으로 갱신됨
  Point2D v = {1.0, 0.0};

  // 기본 종료 이유: max_chain_len 도달 (while 조건에 의한 정상 종료)
  stop_reason = StopReason::MAX_LEN;

  // forward_cone_deg의 반각(half angle)을 라디안으로 변환
  // 예: 120° → 60° → π/3 rad ≈ 1.047 rad
  const double cone_half_rad = cp.forward_cone_deg * M_PI / 360.0;

  // ── Greedy Chaining 메인 루프 ──
  while (static_cast<int>(backbone.size()) < cp.max_chain_len) {

    // ━━━ 1단계: 후보 탐색 ━━━
    // 현재 노드의 kNN 이웃 중 owner==NONE인 노드만 대상
    auto neighbors = knn(points, current, cp.k);

    // ━━━ 2단계: owner 필터 + 3개 게이트 적용 ━━━
    // owner==NONE + G1(거리) + G2(전방 콘) + G3(횡오차) 모두 통과한 후보만 gated에 추가
    std::vector<int> gated;
    bool had_candidates = false;  // NONE 후보가 있었는지 추적

    for (int j : neighbors) {
      // 이미 소유권이 부여된 노드는 건너뜀
      if (owner[j] != NodeOwner::NONE) continue;
      had_candidates = true;  // NONE 후보가 최소 1개 존재

      // 이미 backbone에 포함된 노드는 건너뜀 (순환 방지)
      if (visited_set.count(j)) continue;

      const double dx = points[j].x - points[current].x;
      const double dy = points[j].y - points[current].y;
      const double d = std::sqrt(dx * dx + dy * dy);

      // ── G1: 거리 게이트 ──
      // d_max 초과이거나 거의 같은 위치(d≈0)인 점은 제외
      if (d > cp.d_max || d < 1e-9) continue;

      // ── G2: 전방 콘 게이트 ──
      // 현재 진행 방향 v와 cur→j 방향 u_ij 사이의 각도가
      // cone_half_rad(전방 콘의 반각)를 초과하면 제외
      Point2D u_ij = {dx / d, dy / d};             // cur→j 단위벡터
      double cos_angle = dot2(v, u_ij);             // v·u_ij = cos(θ)
      cos_angle = std::clamp(cos_angle, -1.0, 1.0); // 부동소수점 안전 처리
      if (std::acos(cos_angle) > cone_half_rad) continue;  // 콘 밖 → 제외

      // ── G3: 횡오차 게이트 ──
      // v의 수직 방향(perp)으로의 투영 거리가 lateral_gate 초과하면 제외
      Point2D perp = {-v.y, v.x};                        // v의 왼쪽 수직벡터
      double lat = std::abs(dx * perp.x + dy * perp.y);  // 횡방향 투영 거리
      if (lat > cp.lateral_gate) continue;

      // 3개 게이트 모두 통과 → 유효 후보
      gated.push_back(j);
    }

    // 유효 후보가 없으면 체이닝 종료
    if (gated.empty()) {
      // had_candidates로 종료 이유 구분:
      // - ALL_GATED: NONE 후보는 있었지만 게이트에서 모두 탈락
      //   → 방향이 급변하거나 장애물이 막는 상황
      // - NO_CANDIDATE: NONE 후보 자체가 없음
      //   → 경계의 끝에 도달했거나 모든 이웃이 이미 소유됨
      stop_reason = had_candidates ? StopReason::ALL_GATED
                                   : StopReason::NO_CANDIDATE;
      break;
    }

    // ━━━ 3단계: w' 최소 비용 선택 ━━━
    // 게이트를 통과한 후보들 중 확장 비용 w' = w + λ·C_side 가
    // 가장 낮은 노드를 "다음 backbone 노드"로 선택한다.
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

    // ━━━ 4단계: 이동 (진행 방향 갱신) ━━━
    // 현재→best 방향으로 진행 방향 v를 갱신한다.
    // 다음 스텝의 G2 콘 방향이 이 v를 기준으로 설정된다.
    const double dx = points[best].x - points[current].x;
    const double dy = points[best].y - points[current].y;
    const double d = std::sqrt(dx * dx + dy * dy);
    v = {dx / d, dy / d};  // 새 진행 방향 (단위벡터)

    visited_set.insert(best);      // 방문 표시 (순환 방지)
    backbone.push_back(best);      // backbone에 추가
    current = best;                // 현재 노드 갱신
  }

  return backbone;
}

// ============================================================================
// 3-4단계: Branch 추출 (Backbone 순회 기반 BFS)
// ============================================================================
//
// [목적]
//   backbone에 포함되지 못한 노드들을 backbone에 연결하여
//   가지(branch)로 구성한다. costmap에 빈틈 없는 비용 장벽을 형성한다.
//
// [알고리즘 — Backbone 순회 기반 정방향 BFS]
//
//   backbone 노드를 B0→B1→B2→... 순서로 순회하며:
//
//   1) Bi의 그래프 이웃 중 허용 라벨 노드를 BFS 큐에 추가
//   2) BFS 큐에서 꺼낸 노드의 이웃도 같은 조건으로 확장
//   3) 수집된 노드들이 Bi의 branch가 됨
//   4) 부모(Bi)로부터 거리순 정렬, max_branch_len 제한
//
// [허용 조건]
//   owner[node] == NONE || owner[node] == branch_label
//
//   → 같은 side의 이미 확정된 branch 노드를 BFS로 통과+재연결 가능
//   → 반대 side의 backbone/branch는 차단
//
//   | 단계          | branch_label  | 허용               | 차단                                    |
//   |---------------|---------------|--------------------|-----------------------------------------|
//   | 3. left branch| LEFT_BRANCH   | NONE, LEFT_BRANCH  | LEFT_BACKBONE, RIGHT_BACKBONE           |
//   | 4. right branch| RIGHT_BRANCH | NONE, RIGHT_BRANCH | LEFT_BACKBONE, RIGHT_BACKBONE, LEFT_BRANCH |
//
// [중복 소속]
//   같은 side의 다른 backbone 노드에 이미 소속된 branch 노드도
//   현재 backbone 노드의 branch에 다시 연결할 수 있다.
//   → 각 backbone 노드마다 독립적인 BFS visited를 사용 (backbone-local).
//   → 촘촘한 costmap 장벽 형성에 기여한다.
//
// ============================================================================

std::vector<BranchInfo> DirectionChainer::extract_branches(
  const ChainingGraph & graph,
  const std::vector<ChainPoint> & points,
  const std::vector<int> & backbone_ids,
  std::vector<NodeOwner> & owner,
  NodeOwner branch_label,
  const PlanningParams::Chainer & cp) const
{
  std::vector<BranchInfo> branches;

  // ── backbone 노드를 순서대로 순회하며 BFS로 branch 수집 ──
  for (int b_idx = 0; b_idx < static_cast<int>(backbone_ids.size()); ++b_idx) {
    const int b_node = backbone_ids[b_idx];

    // backbone-local BFS visited: 같은 backbone 내 중복 방문만 방지
    // (다른 backbone의 branch 노드는 다시 방문 가능)
    std::unordered_set<int> bfs_visited;
    std::vector<int> branch_nodes;
    std::queue<int> q;

    // backbone 노드의 그래프 이웃 중 허용 노드를 BFS 시드로 추가
    for (int nb : graph.undirected[b_node]) {
      // 허용 조건: NONE 또는 같은 side의 branch
      if (owner[nb] != NodeOwner::NONE && owner[nb] != branch_label) continue;
      if (bfs_visited.count(nb)) continue;

      bfs_visited.insert(nb);
      branch_nodes.push_back(nb);
      owner[nb] = branch_label;  // 최초 발견 시 라벨 부여
      q.push(nb);
    }

    // BFS 확장: branch 노드의 이웃도 같은 조건으로 탐색
    while (!q.empty()) {
      int cur = q.front();
      q.pop();

      for (int nb : graph.undirected[cur]) {
        if (owner[nb] != NodeOwner::NONE && owner[nb] != branch_label) continue;
        if (bfs_visited.count(nb)) continue;

        bfs_visited.insert(nb);
        branch_nodes.push_back(nb);
        owner[nb] = branch_label;
        q.push(nb);
      }
    }

    if (branch_nodes.empty()) continue;

    // ── 부모(Bi)로부터 거리 순 정렬 ──
    const double px = points[b_node].x;
    const double py = points[b_node].y;
    std::sort(branch_nodes.begin(), branch_nodes.end(),
      [&](int a, int b) {
        double da = (points[a].x - px) * (points[a].x - px) +
                    (points[a].y - py) * (points[a].y - py);
        double db = (points[b].x - px) * (points[b].x - px) +
                    (points[b].y - py) * (points[b].y - py);
        return da < db;
      });

    // ── max_branch_len 제한 ──
    if (static_cast<int>(branch_nodes.size()) > cp.max_branch_len) {
      branch_nodes.resize(cp.max_branch_len);
    }

    // BranchInfo 구성
    BranchInfo bi;
    bi.parent_backbone_idx = b_idx;
    bi.points.reserve(branch_nodes.size());
    for (int idx : branch_nodes) {
      bi.points.push_back(points[idx]);
    }
    bi.score = branch_nodes.empty() ? 0.0 : 1.0 / static_cast<double>(branch_nodes.size());

    branches.push_back(std::move(bi));
  }

  return branches;
}

// ============================================================================
// 6단계: Component 리샘플링 (Resample — 전 edge 보간)
// ============================================================================
//
// [목적]
//   backbone + branch의 모든 edge(연속된 두 점 사이 구간)를
//   resample_ds 간격으로 선형 보간하여 균등한 점열을 생성한다.
//   이 결과가 costmap에 전달되어 Gaussian 비용 장벽이 그려진다.
//
// [왜 리샘플링이 필요한가?]
//
//   원본 경계점 (불균등 간격):
//     ●───────────●──●──────────────────●───●
//     ↑           ↑  ↑                  ↑   ↑
//   가까움    보통 보통     매우 먼 간격    보통
//
//   리샘플링 후 (균등 간격):
//     ●──●──●──●──●──●──●──●──●──●──●──●──●──●
//
//   costmap에서 각 점 주위에 Gaussian 비용을 그리므로,
//   점 간격이 넓으면 비용 장벽에 "구멍"이 생긴다.
//   차량이 이 구멍으로 빠져나가면 경계를 벗어나게 된다.
//   균등 간격 보간으로 연속적인 비용 장벽을 보장한다.
//
// [리샘플 과정]
//
//   1) backbone edge 보간:
//      backbone[0]→backbone[1], backbone[1]→backbone[2], ...
//      각 edge를 resample_ds 간격으로 선형 보간.
//      마지막 backbone 점은 별도 추가.
//
//   2) branch edge 보간:
//      각 branch에 대해:
//      a) 연결 edge: backbone 부모 노드 → branch 첫 번째 점
//      b) 내부 edge: branch[0]→branch[1], branch[1]→branch[2], ...
//      c) 마지막 branch 점은 별도 추가
//
// [보간점의 속성]
//   - type: 양끝이 모두 CONE이면 CONE, 아니면 LANE
//     → costmap에서 콘 구간은 더 높은 비용(cone_cost_max)이 적용됨
//
// ============================================================================

std::vector<ChainPoint> DirectionChainer::resample_component(
  const std::vector<ChainPoint> & points,
  const std::vector<int> & backbone_ids,
  const std::vector<BranchInfo> & branches,
  double resample_ds) const
{
  std::vector<ChainPoint> resampled;
  if (backbone_ids.empty()) return resampled;

  // ── edge 보간 헬퍼 람다 ──
  // 두 점(a, b) 사이를 resample_ds 간격으로 보간하여 resampled에 추가한다.
  // 주의: 시작점 a는 항상 추가하지만, 끝점 b는 추가하지 않는다.
  //       (다음 edge의 시작점으로 추가되거나, 마지막에 별도 추가)
  auto resample_edge = [&](const ChainPoint & a, const ChainPoint & b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len = std::sqrt(dx * dx + dy * dy);  // edge 길이

    // 시작점은 항상 추가
    resampled.push_back(a);

    // edge 길이가 resample_ds 이상이면 중간 보간점 생성
    if (len >= resample_ds) {
      // 보간점의 type 결정:
      // 양끝이 모두 CONE이면 CONE (콘 구간), 아니면 LANE (차선 구간)
      const PointType seg_type =
        (a.type == PointType::CONE && b.type == PointType::CONE)
          ? PointType::CONE : PointType::LANE;

      // resample_ds 간격으로 보간점 생성
      // t = d/len → 0~1 사이의 보간 비율
      for (double d = resample_ds; d < len; d += resample_ds) {
        const double t = d / len;  // 보간 비율 (0: a, 1: b)
        ChainPoint interp;
        interp.x = a.x + t * dx;  // 선형 보간 x
        interp.y = a.y + t * dy;  // 선형 보간 y
        interp.type = seg_type;
        resampled.push_back(interp);
      }
    }
    // 끝점 b는 여기서 추가하지 않음 (다음 edge의 시작점으로 추가됨)
  };

  // ── 1) backbone edges 리샘플 ──
  // backbone의 연속된 점 쌍을 순회하며 보간
  // backbone[0]→[1], [1]→[2], ..., [N-2]→[N-1]
  for (size_t i = 0; i + 1 < backbone_ids.size(); ++i) {
    resample_edge(points[backbone_ids[i]], points[backbone_ids[i + 1]]);
  }
  // backbone 마지막 점은 별도 추가 (resample_edge에서 끝점을 안 넣으므로)
  if (!backbone_ids.empty()) {
    resampled.push_back(points[backbone_ids.back()]);
  }

  // ── 2) 각 branch의 연결 edge + 내부 edges 리샘플 ──
  for (const auto & branch : branches) {
    if (branch.points.empty()) continue;

    // parent_backbone_idx 유효성 검사
    if (branch.parent_backbone_idx < 0 ||
        branch.parent_backbone_idx >= static_cast<int>(backbone_ids.size())) {
      continue;  // 잘못된 인덱스 → 건너뜀
    }

    // 연결 edge: backbone의 부모 노드 → branch 첫 번째 점
    // → backbone에서 branch로 "갈라지는" 구간을 보간
    const auto & parent_pt = points[backbone_ids[branch.parent_backbone_idx]];
    resample_edge(parent_pt, branch.points[0]);

    // branch 내부 edges: branch[0]→[1], [1]→[2], ...
    for (size_t i = 0; i + 1 < branch.points.size(); ++i) {
      resample_edge(branch.points[i], branch.points[i + 1]);
    }
    // branch 마지막 점 별도 추가
    resampled.push_back(branch.points.back());
  }

  return resampled;
}

}  // namespace chaining_costmap_ver
