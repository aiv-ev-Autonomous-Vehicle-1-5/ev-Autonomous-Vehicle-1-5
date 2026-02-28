/**
 * @file direction_chainer.cpp
 * @brief DirectionChainer — 7단계 파이프라인 구현부
 *
 * [파일 구조]
 *   이 파일은 direction_chainer.hpp에 선언된 DirectionChainer 클래스의
 *   모든 멤버 함수를 구현한다. 각 함수는 파이프라인의 한 단계에 대응한다.
 *
 * [7단계 파이프라인 요약]
 *   0단계: preprocess()          — 신뢰도 필터로 노이즈 제거
 *   1단계: find_seed()           — 좌/우 시작점 선택 (콘 우선)
 *   2단계: build_graph()         — kNN + G1,G3 게이트로 undirected 그래프 구성
 *   3단계: extract_component()   — BFS로 seed 연결 성분 추출 (visited 공유)
 *   4단계: extract_backbone()    — greedy chaining으로 주 경계선 추출
 *   5단계: extract_branches()    — 잔여 노드를 backbone에 연결하여 가지 구성
 *   6단계: resample_component()  — 전 edge를 일정 간격으로 보간
 *
 * [의존 관계]
 *   - geometry.hpp: dot2() — 2D 벡터 내적 함수
 *   - types.hpp: ChainPoint, ChainingGraph, BranchInfo, StopReason 등
 *   - params.hpp: PlanningParams::Chainer — 모든 튜닝 파라미터
 */
#include "chaining_CDT/chainer/direction_chainer.hpp"
#include "chaining_CDT/common/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <numeric>

namespace chaining_CDT
{

// ============================================================================
// 메인 chain() — 7단계 파이프라인 통합 실행
// ============================================================================
//
// [전체 흐름 다이어그램]
//
//   입력: ChainPoint[] (콘+차선 혼합)
//     │
//     ▼
//   0단계: preprocess() → 저신뢰 점 제거
//     │
//     ▼
//   1단계: find_seed() × 2 → 좌측/우측 seed 선택
//     │
//     ▼
//   2단계: build_graph() → kNN+게이트 기반 undirected 그래프
//     │
//     ▼
//   3단계: extract_component() × 2 → 좌/우 연결 성분 분리
//     │                                (visited 공유로 중복 방지)
//     ▼
//   4~6단계: process_side() × 2
//     │  ├── 4: extract_backbone() → 주 경계선 추출
//     │  ├── 5: extract_branches() → 잔여 노드 → 가지 구성
//     │  └── 6: resample_component() → 균등 보간
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

  // ── 0단계: 전처리 ──
  // 신뢰도(confidence)가 낮은 점을 제거하여 노이즈/오탐을 사전 차단한다.
  // 필터링 후 점이 2개 미만이면 체이닝 자체가 불가능하므로 조기 반환.
  auto filtered = preprocess(points, cp);
  if (filtered.size() < 2) return result;

  // ── 1단계: Seed 선택 ──
  // 좌측(y>0)과 우측(y<0) 각각에서 ego에 가장 가까운 전방 점을 seed로 선택한다.
  // 양쪽 모두 seed를 찾지 못하면 체이닝이 불가능.
  int left_seed = find_seed(filtered, true, cp);    // 좌측 seed (y > side_seed_y)
  int right_seed = find_seed(filtered, false, cp);   // 우측 seed (y < -side_seed_y)
  if (left_seed < 0 && right_seed < 0) return result;

  // ── 2단계: Undirected Graph 구성 ──
  // 모든 점에 대해 kNN + G1(거리) + G3(횡오차) 게이트를 적용하여
  // 양방향 인접 리스트를 구성한다.
  // 이 그래프는 3단계 BFS와 5단계 branch BFS에서 재사용된다.
  auto graph = build_graph(filtered, cp);

  // ── 3단계: Side Component 추출 (visited 공유) ──
  //
  // 핵심: visited 배열을 좌/우가 공유한다.
  // → 먼저 BFS를 실행한 쪽이 차지한 노드는 반대쪽이 접근할 수 없다.
  // → 하나의 점이 좌/우 양쪽에 배정되는 것을 원천 방지한다.
  //
  // ego에 더 가까운 seed를 먼저 처리한다.
  // 이유: ego에 가까운 점은 좌/우 구분이 더 명확하므로,
  //       가까운 쪽이 먼저 확실한 노드들을 차지하게 한다.
  std::vector<bool> visited(filtered.size(), false);

  // 좌/우 seed의 ego 거리 계산 (어느 쪽을 먼저 처리할지 결정)
  double left_dist = (left_seed >= 0)
    ? std::sqrt(filtered[left_seed].x * filtered[left_seed].x +
                filtered[left_seed].y * filtered[left_seed].y)
    : std::numeric_limits<double>::max();   // seed 없으면 ∞ → 나중에 처리
  double right_dist = (right_seed >= 0)
    ? std::sqrt(filtered[right_seed].x * filtered[right_seed].x +
                filtered[right_seed].y * filtered[right_seed].y)
    : std::numeric_limits<double>::max();

  std::vector<int> l_component, r_component;

  // ego에 더 가까운 seed를 먼저 BFS 실행 (visited 선점)
  if (left_dist <= right_dist) {
    if (left_seed >= 0)
      l_component = extract_component(graph, left_seed, visited);
    if (right_seed >= 0)
      r_component = extract_component(graph, right_seed, visited);
  } else {
    if (right_seed >= 0)
      r_component = extract_component(graph, right_seed, visited);
    if (left_seed >= 0)
      l_component = extract_component(graph, left_seed, visited);
  }

  // ── 디버그: 리샘플 전 좌/우 component별 콘/차선 개수 출력 ──
  if (cp.debug_chainer_stats) {
    auto count_types = [&](const std::vector<int> & comp, const char * label) {
      int n_cone = 0, n_lane = 0;
      for (int idx : comp) {
        if (filtered[idx].type == PointType::CONE) ++n_cone;
        else ++n_lane;
      }
      RCLCPP_INFO(rclcpp::get_logger("direction_chainer"),
        "[%s] pre-resample: total=%d  cones=%d  lanes=%d",
        label, static_cast<int>(comp.size()), n_cone, n_lane);
    };
    count_types(l_component, "LEFT");
    count_types(r_component, "RIGHT");
  }

  // ── 4~6단계: 각 side별 backbone/branch/resample 처리 ──
  //
  // 좌/우 각각 동일한 과정을 거친다:
  //   4단계: backbone 추출 (greedy chaining으로 주 경계선)
  //   5단계: branch 추출 (잔여 노드를 backbone에 연결)
  //   6단계: resample (전 edge를 균등 간격으로 보간)
  //
  // 이 과정을 lambda로 정의하여 좌/우에 동일하게 적용한다.
  auto process_side = [&](
    const std::vector<int> & comp_ids,
    int seed_idx,
    bool is_left) -> SideResult
  {
    SideResult side;
    if (comp_ids.empty() || seed_idx < 0) return side;

    side.seed_idx = seed_idx;

    // ━━━ 4단계: Backbone 추출 ━━━
    // seed에서 출발하여 greedy하게 전방으로 확장.
    // 비용함수 w' = w + λ·C_side 최소인 다음 점을 선택한다.
    // stop_reason에 종료 이유가 기록된다
    // (후보 없음 / 게이트 탈락 / max_len 도달).
    StopReason stop;
    auto backbone_ids = extract_backbone(
      filtered, comp_ids, seed_idx, is_left, stop, cp);
    side.stop_reason = stop;

    if (backbone_ids.empty()) return side;

    // goal_idx: backbone의 마지막 노드 (체이닝이 끝난 지점)
    side.goal_idx = backbone_ids.back();

    // backbone 포인트를 SideResult에 복사 (디버그/시각화용)
    side.backbone.reserve(backbone_ids.size());
    for (int idx : backbone_ids) {
      side.backbone.push_back(filtered[idx]);
    }

    // ━━━ 5단계: Branch 추출 ━━━
    // branch_mode가 "backbone_and_branches"일 때만 실행.
    // 잔여 노드를 BFS로 가장 가까운 backbone 노드에 연결하여
    // 가지(branch)를 구성한다.
    if (cp.branch_mode == "backbone_and_branches") {
      side.branches = extract_branches(
        graph, filtered, comp_ids, backbone_ids, cp);
    }

    // ━━━ 6단계: Component 리샘플링 ━━━
    // backbone + branch의 모든 edge를 resample_ds 간격으로 보간한다.
    // 결과: costmap에 전달할 균등 간격 경계점열.
    side.component = resample_component(
      filtered, backbone_ids, side.branches, cp.resample_ds);

    return side;
  };

  // 좌/우 각각 4~6단계 실행
  result.left = process_side(l_component, left_seed, true);    // 좌측 처리
  result.right = process_side(r_component, right_seed, false);  // 우측 처리

  // 최소 한쪽 backbone이 생성되었으면 유효한 결과
  result.valid = (!result.left.backbone.empty() ||
                  !result.right.backbone.empty());
  return result;
}

// ============================================================================
// 0단계: 전처리 (Preprocess)
// ============================================================================
//
// [목적]
//   BBox 검출기나 차선 인식기가 출력한 경계점 중 신뢰도가 낮은 것을 제거한다.
//   신뢰도가 낮은 점은 오탐(false positive)일 가능성이 높으므로,
//   그래프에 포함시키면 잘못된 연결이 생길 수 있다.
//
// [동작]
//   - confidence >= min_confidence 인 점만 새 배열에 복사
//   - 인덱스가 재배열됨 (이후 단계는 이 필터된 배열의 인덱스 사용)
//
// [파라미터]
//   min_confidence: 최소 신뢰도 컷오프 (기본값 0.1)
//   → 너무 높게 설정하면 유효한 점도 제거됨 → 경계 누락
//   → 너무 낮게 설정하면 노이즈가 그대로 통과 → 잘못된 체이닝
//
// ============================================================================

std::vector<ChainPoint> DirectionChainer::preprocess(
  const std::vector<ChainPoint> & points,
  const PlanningParams::Chainer & cp) const
{
  std::vector<ChainPoint> out;
  out.reserve(points.size());  // 메모리 미리 확보 (재할당 방지)
  for (const auto & p : points) {
    // 신뢰도 필터: min_confidence 미만인 점은 제외
    if (p.confidence >= cp.min_confidence) {
      out.push_back(p);
    }
  }
  return out;
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
//   1) 전방 필터: x ≥ 0 인 점만 후보
//      → 후방 점에서 시작하면 체인이 뒤로 뻗어 무의미
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
//
//   4) cone_priority: 콘이 있으면 콘을 우선 seed로 사용
//      → 콘(PE 드럼)은 크기가 일정하고 위치 정확도가 높음
//      → 차선점보다 안정적인 시작점이 됨
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
  int best_cone = -1;      // 콘 중 최근접 후보 인덱스 (cone_priority용)
  double best_dist_sq = std::numeric_limits<double>::max();       // 최소 거리² (전체)
  double best_cone_dist_sq = std::numeric_limits<double>::max();  // 최소 거리² (콘)
  const int n = static_cast<int>(points.size());

  for (int i = 0; i < n; ++i) {
    // x < -1 (후방)인 점은 seed 후보에서 제외
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

  //   // cone_priority 모드: 콘 중 최근접도 별도로 추적
  //   // → 최종 선택 시 콘이 있으면 콘을 우선 반환
  //   if (cp.cone_priority && points[i].type == PointType::CONE) {
  //     if (d_sq < best_cone_dist_sq) {
  //       best_cone_dist_sq = d_sq;
  //       best_cone = i;
  //     }
  //   }
  // }

  // // 콘 후보가 있으면 콘을 우선 seed로 선택 (위치 정확도가 높으므로)
  // if (cp.cone_priority && best_cone >= 0) {
  //   return best_cone;
  // }
  // 콘이 없으면 전체 최근접 점을 seed로 사용 (차선점 포함)
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
// 3단계: Component 추출 (BFS)
// ============================================================================
//
// [목적]
//   seed에서 출발하여 그래프 상에서 연결된 모든 노드를 하나의 "component"로 묶는다.
//   component = 같은 방향(좌 또는 우) 경계에 속하는 점들의 집합.
//
// [BFS (너비 우선 탐색) 동작]
//   1. seed를 큐에 넣고 visited 표시
//   2. 큐에서 하나 꺼내서 인접 노드 확인
//   3. 미방문 인접 노드를 큐에 넣고 visited 표시 + component에 추가
//   4. 큐가 빌 때까지 반복
//
// [visited 공유의 핵심 효과]
//   visited 배열은 chain() 함수에서 좌/우 양쪽이 공유한다.
//   따라서:
//   - 좌측 BFS가 먼저 실행되면, 좌측 seed와 연결된 노드들이 visited=true 됨
//   - 이후 우측 BFS 실행 시, 이미 visited된 노드는 접근 불가
//   - 결과: 하나의 점이 좌/우 양쪽 component에 중복 배정되지 않음
//
//   이 메커니즘이 "ego에 가까운 seed를 먼저 처리"하는 이유와 결합된다:
//   ego 근처에서는 좌/우 구분이 명확하므로, 확실한 쪽이 먼저 점유한다.
//
// [경계 사례]
//   - seed가 유효하지 않거나(-1, 범위 초과) → 빈 배열 반환
//   - seed가 이미 visited → 빈 배열 반환 (반대쪽이 이미 차지함)
//
// ============================================================================

std::vector<int> DirectionChainer::extract_component(
  const ChainingGraph & graph,
  int seed_idx,
  std::vector<bool> & visited) const
{
  std::vector<int> component;

  // seed 유효성 검사: 인덱스 범위 / 이미 방문 여부
  if (seed_idx < 0 ||
      seed_idx >= static_cast<int>(graph.undirected.size()) ||
      visited[seed_idx]) {
    return component;  // 빈 배열 반환
  }

  // BFS 시작: seed를 큐에 넣고 방문 표시
  std::queue<int> q;
  q.push(seed_idx);
  visited[seed_idx] = true;
  component.push_back(seed_idx);

  // BFS 탐색: 큐가 빌 때까지 반복
  while (!q.empty()) {
    int cur = q.front();
    q.pop();

    // 현재 노드의 모든 인접 노드를 확인
    for (int nb : graph.undirected[cur]) {
      if (visited[nb]) continue;  // 이미 방문 → 건너뜀 (좌/우 공유 visited)
      visited[nb] = true;          // 방문 표시 (반대쪽 BFS가 접근 못하게)
      component.push_back(nb);     // component에 추가
      q.push(nb);                  // 큐에 넣어 후속 탐색
    }
  }

  return component;
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
// 4단계: Backbone 추출 (Greedy Chaining)
// ============================================================================
//
// [목적]
//   component 내에서 seed부터 전방으로 이어지는 "주 경계선"(backbone)을 추출한다.
//   이 backbone이 한쪽 경계의 핵심 구조이다.
//
// [Greedy Chaining 알고리즘]
//
//   seed ──→ ● ──→ ● ──→ ● ──→ ● ──→ goal
//            ↑
//         매 스텝마다:
//         1) 현재 노드의 kNN 후보 탐색
//         2) 3개 게이트(G1+G2+G3) 적용하여 부적합 후보 제거
//         3) 통과한 후보들의 w' 비용 계산
//         4) w' 최소인 노드로 이동, 진행 방향 갱신
//         5) 반복 (종료 조건 충족 시 중단)
//
// [3개 게이트 — backbone 전용 (2단계 graph와 다른 점)]
//
//   G1 (거리 게이트): d(cur, j) ≤ d_max  AND  d > 0
//     → 2단계와 동일. 너무 먼 점 차단.
//     → d < 1e-9: 거의 같은 위치의 점 제외 (방향 계산 불가)
//
//   G2 (전방 콘 게이트): angle(v, u_ij) ≤ cone_half_rad  ★ 2단계에는 없음!
//     → 현재 진행 방향 v 기준으로 "전방 콘" 안에 있는 점만 통과
//     → 뒤쪽/완전 옆쪽 점은 "다음 경계점"이 아님
//     → 예: forward_cone_deg=120° → 전방 ±60° 범위만 허용
//
//         v (진행방향)
//          \  ±60°  /
//           \      /
//            \    /   ← 이 콘 안의 점만 후보
//             \  /
//              ● (현재 노드)
//
//   G3 (횡오차 게이트): |lat_proj| ≤ lateral_gate
//     → 진행 방향 v의 수직 성분으로 투영한 거리가 lateral_gate 이내
//     → 2단계의 단순 |Δy|와 달리, 진행 방향 기준의 정밀한 횡오차
//
// [종료 조건과 StopReason]
//   - MAX_LEN: max_chain_len 도달 (정상 종료, 충분히 길게 체이닝됨)
//   - NO_CANDIDATE: kNN 중 component 내 후보가 하나도 없음 (경계 끝)
//   - ALL_GATED: 후보는 있지만 게이트를 모두 탈락 (방향 급변/장애물)
//
// [진행 방향 v 갱신]
//   - 초기값: v = (1,0) — base_link 전방
//   - 매 스텝: v = (dx/d, dy/d) — 현재→다음 방향으로 갱신
//   - 이렇게 하면 G2 콘이 체인의 진행 방향을 따라간다
//
// ============================================================================

std::vector<int> DirectionChainer::extract_backbone(
  const std::vector<ChainPoint> & points,
  const std::vector<int> & component_ids,
  int seed_idx,
  bool is_left,
  StopReason & stop_reason,
  const PlanningParams::Chainer & cp) const
{
  // component 소속 여부를 O(1)로 검색하기 위한 hash set
  std::unordered_set<int> comp_set(component_ids.begin(), component_ids.end());

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
    // 현재 노드의 kNN 이웃 중 component에 속하는 노드만 대상
    auto neighbors = knn(points, current, cp.k);

    // ━━━ 2단계: 3개 게이트 적용 ━━━
    // G1(거리) + G2(전방 콘) + G3(횡오차) 모두 통과한 후보만 gated에 추가
    std::vector<int> gated;
    bool had_candidates = false;  // component 내 후보가 있었는지 추적

    for (int j : neighbors) {
      // component 밖의 노드는 건너뜀
      if (!comp_set.count(j)) continue;
      had_candidates = true;  // component 내에 후보가 최소 1개 존재

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
      // - ALL_GATED: component 내 후보는 있었지만 게이트에서 모두 탈락
      //   → 방향이 급변하거나 장애물이 막는 상황
      // - NO_CANDIDATE: component 내 후보 자체가 없음
      //   → 경계의 끝에 도달했거나 component가 작음
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
// 5단계: Branch 추출 (잔여 노드 → Backbone 연결)
// ============================================================================
//
// [목적]
//   backbone(주 경계선)에 포함되지 못한 "잔여 노드"들을
//   backbone에 연결하여 가지(branch)로 구성한다.
//
// [왜 Branch가 필요한가?]
//
//   backbone은 seed→전방 방향으로 greedy하게 뻗어나가므로,
//   component 내 모든 점을 포함하지 못할 수 있다:
//
//     backbone:  seed ──→ ● ──→ ● ──→ ● ──→ goal
//                              ↑
//                          ● ← 잔여 노드 (backbone 옆에 있지만 선택 안 됨)
//                          ●
//                          ● ← 이 점들도 경계 정보를 담고 있다!
//
//   이런 잔여 노드를 무시하면 costmap에 "구멍"이 생긴다.
//   branch로 연결하면 경계 정보를 최대한 활용할 수 있다.
//
// [알고리즘]
//
//   1) 잔여 노드 수집:
//      component에 속하지만 backbone에는 없는 노드들
//
//   2) 각 잔여 노드에서 BFS → 가장 가까운 backbone 노드 탐색:
//      undirected 그래프 위에서 BFS를 실행하여
//      최초로 만나는 backbone 노드를 "부모(parent)"로 결정한다.
//      BFS이므로 최단 hop 거리의 backbone 노드가 선택된다.
//
//   3) 같은 부모를 공유하는 잔여 노드들을 묶어 하나의 branch 구성:
//      backbone 상의 같은 지점에서 분기되는 점들의 그룹.
//
//   4) 부모로부터 거리 순 정렬:
//      branch 내 점들을 부모에서 가까운 순서로 정렬한다.
//      → 6단계 리샘플링에서 edge를 올바른 순서로 보간하기 위함.
//
//   5) max_branch_len 제한:
//      너무 긴 branch는 잘라낸다 (노이즈/오탐 방지).
//
// [BranchInfo 구조]
//   - parent_backbone_idx: backbone 상의 분기점 위치 (0-based 인덱스)
//   - points: branch 포인트들 (부모에서 가까운 순)
//   - score: branch 품질 (현재는 1/N, N=점 수)
//
// ============================================================================

std::vector<BranchInfo> DirectionChainer::extract_branches(
  const ChainingGraph & graph,
  const std::vector<ChainPoint> & points,
  const std::vector<int> & component_ids,
  const std::vector<int> & backbone_ids,
  const PlanningParams::Chainer & cp) const
{
  // backbone 소속 여부를 O(1)로 검색하기 위한 hash set
  std::unordered_set<int> backbone_set(backbone_ids.begin(), backbone_ids.end());

  // point index → backbone 내 순서 매핑
  // 예: backbone_ids = [5, 12, 3] → {5→0, 12→1, 3→2}
  // → BranchInfo.parent_backbone_idx에 backbone "몇 번째 노드"인지 기록할 때 사용
  std::unordered_map<int, int> backbone_idx_map;
  for (int i = 0; i < static_cast<int>(backbone_ids.size()); ++i) {
    backbone_idx_map[backbone_ids[i]] = i;
  }

  // ── 1) 잔여 노드 수집 ──
  // component에 속하지만 backbone에는 없는 노드들
  std::vector<int> rest;
  for (int id : component_ids) {
    if (!backbone_set.count(id)) {
      rest.push_back(id);
    }
  }

  // 잔여 노드가 없으면 branch도 없음
  if (rest.empty()) return {};

  // ── 2) 각 잔여 노드 → 가장 가까운 backbone 노드 탐색 (BFS) ──
  // attach: 잔여 노드 → 부모 backbone 노드 매핑
  std::unordered_map<int, int> attach;

  for (int n : rest) {
    // 잔여 노드 n에서 BFS 시작 → backbone 노드를 최초로 만나면 부모로 결정
    std::queue<int> q;
    std::unordered_set<int> bfs_visited;
    q.push(n);
    bfs_visited.insert(n);
    int found_parent = -1;

    while (!q.empty() && found_parent < 0) {
      int cur = q.front();
      q.pop();

      for (int nb : graph.undirected[cur]) {
        if (bfs_visited.count(nb)) continue;
        bfs_visited.insert(nb);

        // backbone 노드를 발견하면 → 이것이 부모
        if (backbone_set.count(nb)) {
          found_parent = nb;
          break;  // BFS이므로 최단 hop 거리의 backbone 노드
        }
        q.push(nb);
      }
    }

    // 부모를 찾았으면 매핑 저장
    if (found_parent >= 0) {
      attach[n] = found_parent;
    }
    // 부모를 못 찾은 경우: 그래프 상 backbone과 연결 안 됨 → 무시
  }

  // ── 3) 같은 부모를 공유하는 잔여 노드들을 묶어 branch 구성 ──
  std::unordered_map<int, std::vector<int>> parent_groups;
  for (const auto & [node, parent] : attach) {
    parent_groups[parent].push_back(node);
  }

  std::vector<BranchInfo> branches;

  for (auto & [parent, nodes] : parent_groups) {
    // ── 4) 부모로부터 거리 순 정렬 ──
    // 유클리드 거리 기준 오름차순 (부모에 가까운 점이 앞)
    const double px = points[parent].x;
    const double py = points[parent].y;
    std::sort(nodes.begin(), nodes.end(),
      [&](int a, int b) {
        double da = (points[a].x - px) * (points[a].x - px) +
                    (points[a].y - py) * (points[a].y - py);
        double db = (points[b].x - px) * (points[b].x - px) +
                    (points[b].y - py) * (points[b].y - py);
        return da < db;  // 거리² 비교 (오름차순)
      });

    // ── 5) max_branch_len 제한 ──
    // 너무 긴 branch는 잘라냄 (노이즈/오탐이 연쇄적으로 연결된 경우 방지)
    if (static_cast<int>(nodes.size()) > cp.max_branch_len) {
      nodes.resize(cp.max_branch_len);
    }

    // BranchInfo 구성
    BranchInfo bi;
    bi.parent_backbone_idx = backbone_idx_map[parent];  // backbone 내 순서
    bi.points.reserve(nodes.size());
    for (int idx : nodes) {
      bi.points.push_back(points[idx]);  // 실제 포인트 데이터 복사
    }
    // score: branch 품질 (점이 적을수록 높음 — 간결한 branch 선호)
    // 현재는 단순히 1/N, 향후 비용 기반 스코어링으로 확장 가능
    bi.score = (nodes.empty()) ? 0.0 : 1.0 / static_cast<double>(nodes.size());

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
//   - confidence: 양끝 중 낮은 값 사용 (보수적 전략)
//     → 한쪽이라도 불확실하면 보간점도 불확실하게 표시
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
        // 신뢰도: 양끝 중 낮은 값 (보수적)
        interp.confidence = std::min(a.confidence, b.confidence);
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

}  // namespace chaining_CDT
