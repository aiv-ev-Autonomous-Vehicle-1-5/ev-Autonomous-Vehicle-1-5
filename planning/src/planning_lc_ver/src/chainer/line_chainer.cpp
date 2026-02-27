/**
 * @file line_chainer.cpp
 * @brief LineChainer — 구현부 (Flood Fill 기반 경계점 체이닝)
 *
 * ══════════════════════════════════════════════════════════════
 *                     구현부 전체 개요
 * ══════════════════════════════════════════════════════════════
 *
 * 이 파일은 LineChainer의 핵심 로직을 구현한다.
 * 두 개의 함수로 구성된다:
 *
 *   1. chain()          — 메인 함수: seed 선택 + visited 생성 + 좌/우 호출
 *   2. chain_one_side() — 핵심 함수: BFS flood fill + 트리 추출 + 리샘플링
 *
 * ── Flood Fill이란? ──
 *   그래프 탐색의 일종으로, 시작점(seed)에서 인접한 점들을 하나씩
 *   "감염"시키며 연결된 영역을 확장해 나가는 알고리즘이다.
 *   BFS(너비 우선 탐색)를 사용하므로, seed에서 가까운 점부터 순서대로
 *   감염된다. 이미지 편집기의 "페인트 버킷(채우기)" 도구와 같은 원리이다.
 *
 *   여기서는 "인접"의 기준이 좌표 거리(search_radius)이다:
 *   현재 점에서 반경 r 이내에 있는 미방문 점이 "이웃"이 된다.
 *
 * ── 왜 Flood Fill을 쓰는가? ──
 *   경계점들이 정렬되지 않은 무작위 순서로 들어온다.
 *   단순 정렬(x좌표 등)로는 곡선이나 분기점을 처리할 수 없다.
 *   Flood fill은 공간적으로 연결된 점들을 자연스럽게 하나의 체인으로
 *   묶어주므로, 곡선/분기 구간에서도 올바른 체이닝이 가능하다.
 *
 * ── 핵심 아이디어 요약 ──
 *   1. y>0 최근접점을 left seed, y<0 최근접점을 right seed로 선택
 *   2. 각 seed에서 flood fill — 탐색 반경 내 모든 점을 BFS로 감염
 *   3. visited 배열을 좌/우가 공유 → 한 점이 양쪽에 중복 소속되는 것을 방지
 *   4. Cone priority: 반경 내 콘+차선 혼합 시 차선 점을 스킵
 *   5. BFS 중 parent→child edge를 기록하여 감염 트리를 구축
 *   6. 감염 트리에서 모든 seed→leaf 경로 추출
 *   7. 각 경로를 resample_ds 간격으로 보간(리샘플링)
 *
 * ══════════════════════════════════════════════════════════════
 */
#include "planning_lc_ver/chainer/line_chainer.hpp"

#include <algorithm>   // std::reverse
#include <cmath>       // std::sqrt
#include <limits>      // std::numeric_limits
#include <queue>       // std::queue (BFS 큐)

namespace planning_lc_ver
{

// ══════════════════════════════════════════════════════════════
//                chain_one_side() — 핵심 함수
// ══════════════════════════════════════════════════════════════
//
// 한쪽(좌 또는 우) 경계를 flood fill로 구축한다.
//
// ── 단계별 상세 설명 ──
//
//  [Phase 1: BFS Flood Fill]
//   Step 1. seed를 visited=true로 마킹, chain 배열에 추가, BFS 큐에 push
//   Step 2. 큐에서 점 하나(cur)를 pop
//   Step 3. cur 중심으로 반경 r 이내의 미방문 이웃을 전부 수집
//           - 거리 계산: dx*dx + dy*dy (제곱 거리 비교로 sqrt 회피)
//           - d_sq < 1e-12 이면 자기 자신이므로 스킵
//   Step 4. [Cone Priority] 수집된 이웃 중 콘과 차선이 혼재하면
//           차선 점은 스킵하고 콘 점만 감염시킨다.
//           → 콘은 물리적 장애물이므로 경계 기준으로 더 신뢰성이 높다.
//   Step 5. [탐색 반경 확장] 이웃이 비어있으면(아무도 못 찾으면)
//           반경을 search_radius_step만큼 늘려서 다시 탐색한다.
//           → search_radius_max까지 확장 가능.
//           → 이웃을 찾으면 즉시 break로 확장 루프를 탈출한다.
//           ※ 확장 이유: 점 밀도가 불균일한 영역에서 끊김을 방지하기 위함.
//              콘 사이 간격이 넓은 구간에서는 기본 반경으로 못 찾을 수 있다.
//   Step 6. 찾은 이웃을 전부 visited=true + chain에 추가 + 큐에 push
//           + parent→child edge를 기록한다.
//   Step 7. 큐가 빌 때까지 Step 2~6 반복
//
//  [Phase 2: 트리 → 경로 추출 + 리샘플링]
//   Step 8. edges 배열로 parent/children 관계를 복원한다.
//   Step 9. child가 없는 노드(leaf)를 수집한다.
//   Step 10. 각 leaf에서 parent를 따라 seed(root)까지 역추적한다.
//   Step 11. 역추적 결과를 뒤집어 seed→leaf 순서의 경로를 얻는다.
//   Step 12. 경로의 각 구간(edge)에 resample_ds 간격으로 보간점을 삽입한다.
//            → 양 끝점 모두 CONE이면 보간점도 CONE, 아니면 LANE
//
// ══════════════════════════════════════════════════════════════
std::vector<ChainedPoint> LineChainer::chain_one_side(
  const std::vector<ChainedPoint> & candidates,
  int seed_idx,
  std::vector<bool> & visited,
  const PlanningParams & params)
{
  // ──────────────────────────────────────────────────────────
  //  초기 설정
  // ──────────────────────────────────────────────────────────
  std::vector<ChainedPoint> chain;   // 감염된 점들을 순서대로 저장할 배열
  const int n = static_cast<int>(candidates.size());

  // 유효하지 않은 seed 인덱스이면 빈 체인을 반환
  if (seed_idx < 0 || seed_idx >= n) return chain;

  const auto & cp = params.chainer;  // 체이너 관련 파라미터 바로가기

  // ──────────────────────────────────────────────────────────
  //  cand_to_chain: candidates 인덱스 → chain 인덱스 매핑 테이블
  // ──────────────────────────────────────────────────────────
  //  candidates[i]가 chain[j]에 대응한다면 cand_to_chain[i] = j
  //  아직 chain에 추가되지 않은 점은 -1이다.
  //  이 매핑은 BFS 중 parent→child edge를 chain 인덱스 기준으로
  //  기록하기 위해 필요하다.
  std::vector<int> cand_to_chain(n, -1);

  // ──────────────────────────────────────────────────────────
  //  [Phase 1 - Step 1] Seed 등록
  // ──────────────────────────────────────────────────────────
  //  seed를 감염(visited=true)시키고, chain의 첫 번째 원소로 추가한다.
  //  seed는 감염 트리의 root 노드가 된다 (chain index = 0).
  visited[seed_idx] = true;
  chain.push_back(candidates[seed_idx]);
  cand_to_chain[seed_idx] = 0;   // chain[0] = seed

  // ──────────────────────────────────────────────────────────
  //  edges: 감염 트리의 간선(edge)을 기록하는 배열
  // ──────────────────────────────────────────────────────────
  //  각 원소는 (parent의 chain idx, child의 chain idx) 쌍이다.
  //  BFS 중 새 점을 감염시킬 때마다 "누가 감염시켰는지(parent)"를
  //  기록한다. 나중에 이 정보로 트리를 복원한다.
  std::vector<std::pair<int, int>> edges;

  // BFS 큐: candidates 배열의 인덱스를 저장한다.
  std::queue<int> q;
  q.push(seed_idx);

  // ──────────────────────────────────────────────────────────
  //  [Phase 1 - Step 2~7] BFS Flood Fill 메인 루프
  // ──────────────────────────────────────────────────────────
  //  큐에서 점을 하나씩 꺼내어, 그 주변 이웃을 감염시킨다.
  //  큐가 빌 때까지(= 더 이상 감염시킬 이웃이 없을 때까지) 반복한다.
  while (!q.empty()) {
    // ── [Step 2] 큐에서 현재 점(cur)을 꺼낸다 ──
    const int cur = q.front();
    q.pop();

    const double cx = candidates[cur].x;   // 현재 점의 x좌표
    const double cy = candidates[cur].y;   // 현재 점의 y좌표
    const int cur_chain_idx = cand_to_chain[cur];  // 현재 점의 chain 인덱스

    // ──────────────────────────────────────────────────────
    //  [Step 3~5] 탐색 반경 확장 루프
    // ──────────────────────────────────────────────────────
    //  초기 반경(search_radius)에서 시작하여, 이웃을 못 찾으면
    //  search_radius_step만큼 반경을 늘려 재시도한다.
    //  search_radius_max까지 확장 가능하며, 이웃을 찾으면 break.
    //
    //  ※ 왜 확장하는가?
    //    점 밀도가 균일하지 않다. 콘 간격이 넓은 구간이나,
    //    차선이 끊긴 구간에서는 기본 반경으로 이웃을 못 찾을 수 있다.
    //    이때 반경을 조금씩 늘려서 연결이 끊기지 않도록 한다.
    //    단, 너무 넓히면 반대쪽 경계 점까지 감염시킬 수 있으므로
    //    search_radius_max로 상한을 둔다.
    //
    //  ※ +1e-6은 부동소수점 비교 오차를 보정하기 위한 엡실론이다.
    for (double r = cp.search_radius; r <= cp.search_radius_max + 1e-6;
         r += cp.search_radius_step)
    {
      const double r_sq = r * r;  // 제곱 비교 (sqrt 계산 회피)

      // ── [Step 3] 반경 r 이내의 미방문 이웃 수집 ──
      //
      //  Neighbor 구조체: 이웃의 candidates 인덱스 + 콘 여부
      //  has_cone / has_lane: 이 반경에서 콘/차선이 각각 존재하는지 추적
      //  → 나중에 Cone Priority 판정에 사용한다.
      struct Neighbor { int idx; bool is_cone; };
      std::vector<Neighbor> neighbors;
      bool has_cone = false;     // 이 반경에 콘이 하나라도 있는가?
      bool has_lane = false;     // 이 반경에 차선이 하나라도 있는가?

      for (int i = 0; i < n; ++i) {
        if (visited[i]) continue;  // 이미 감염된 점은 스킵

        const double dx = candidates[i].x - cx;   // x방향 거리
        const double dy = candidates[i].y - cy;   // y방향 거리
        const double d_sq = dx * dx + dy * dy;    // 제곱 거리

        // d_sq < 1e-12: 사실상 같은 위치 (자기 자신 또는 중복점) → 스킵
        // d_sq > r_sq:  반경 밖 → 스킵
        if (d_sq < 1e-12 || d_sq > r_sq) continue;

        // 이웃으로 확정 → 타입(콘/차선)을 기록하여 추가
        const bool cone = (candidates[i].type == PointType::CONE);
        neighbors.push_back({i, cone});
        if (cone) has_cone = true;   // 콘 존재 플래그 세팅
        else      has_lane = true;   // 차선 존재 플래그 세팅
      }

      // ── [Step 5] 이웃이 없으면 → continue로 반경 확장 ──
      //  for 루프의 r += search_radius_step에 의해 반경이 늘어난다.
      if (neighbors.empty()) continue;

      // ──────────────────────────────────────────────────────
      //  [Step 4 + 6] 감염: 이웃을 chain에 추가 + 큐에 push
      // ──────────────────────────────────────────────────────
      //  이 단계에서 수집된 이웃들을 실제로 감염시킨다.
      //  Cone Priority가 여기서 적용된다.
      for (const auto & nb : neighbors) {
        // ── Cone Priority (콘 우선 정책) ──
        //  has_cone && has_lane: 이 반경에 콘과 차선이 섞여 있다.
        //  이 경우, 차선 점(!nb.is_cone)은 스킵하고 콘 점만 감염시킨다.
        //
        //  ※ 왜 콘을 우선하는가?
        //    콘(PE 드럼)은 물리적 장애물이므로, 경로 경계를 결정하는 데
        //    차선보다 더 확실한 기준점이 된다.
        //    차선 점이 콘 근처에 있으면, 차선은 콘 사이를 보충하는 용도로
        //    나중에 사용되어야 하므로 이 단계에서는 건너뛴다.
        //
        //  ※ 차선만 있으면? → has_cone=false이므로 조건 불성립, 차선도 감염됨.
        //  ※ 콘만 있으면?  → has_lane=false이므로 조건 불성립, 콘이 감염됨.
        //  즉, "혼합"일 때만 차선을 스킵하는 것이다.
        if (has_cone && has_lane && !nb.is_cone) continue;

        // 감염 처리: visited + chain + 매핑 + edge + 큐
        visited[nb.idx] = true;    // ① 방문 플래그 세팅 (좌/우 공유!)
        const int child_chain_idx = static_cast<int>(chain.size());
        chain.push_back(candidates[nb.idx]);       // ② chain에 추가
        cand_to_chain[nb.idx] = child_chain_idx;   // ③ 매핑 테이블 갱신
        edges.push_back({cur_chain_idx, child_chain_idx});  // ④ 트리 edge 기록
        q.push(nb.idx);            // ⑤ BFS 큐에 추가 (다음 턴에 이 점 주변 탐색)
      }

      // 이 반경에서 이웃을 찾았으므로 확장 루프를 탈출한다.
      // (더 넓은 반경으로 탐색할 필요 없음)
      break;
    }
  }
  // ── Phase 1 종료: BFS 완료 ──
  // 이 시점에서 chain 배열에는 감염된 모든 점이 들어 있고,
  // edges 배열에는 감염 트리의 모든 간선이 기록되어 있다.

  // ══════════════════════════════════════════════════════════
  //  [Phase 2] 감염 트리 → 경로(Polyline) 추출 + 리샘플링
  // ══════════════════════════════════════════════════════════
  //
  //  Phase 1에서 구축된 감염 트리의 구조:
  //
  //          seed (root, chain idx 0)
  //          / | \
  //        c1  c2  c3       ← seed에서 직접 감염된 자식들
  //        |       |
  //        c4      c5
  //       / \
  //     c6   c7 (leaf)      ← 자식이 없는 말단 노드
  //    (leaf)
  //
  //  이 트리에서 모든 root→leaf 경로를 추출한다.
  //  위 예시라면:
  //    경로 1: seed → c1 → c4 → c6
  //    경로 2: seed → c1 → c4 → c7
  //    경로 3: seed → c2 (c2가 leaf인 경우)
  //    경로 4: seed → c3 → c5 (c5가 leaf인 경우)

  const double ds = cp.resample_ds;   // 리샘플링 간격 [m]
  const int cn = static_cast<int>(chain.size());   // 감염된 총 점 수

  // ──────────────────────────────────────────────────────────
  //  [Step 8] parent 배열 + child_count 배열 구축
  // ──────────────────────────────────────────────────────────
  //  parent[i]: chain[i]의 부모 노드의 chain 인덱스 (-1이면 root)
  //  child_count[i]: chain[i]의 자식 수 (0이면 leaf 노드)
  //
  //  edges 배열의 각 원소 {a, b}는 "a가 b를 감염시켰다"를 의미하므로,
  //  parent[b] = a, child_count[a]++ 로 복원한다.
  std::vector<int> parent(cn, -1);       // 모든 노드의 부모를 -1(root)로 초기화
  std::vector<int> child_count(cn, 0);   // 모든 노드의 자식 수를 0으로 초기화
  for (const auto & edge : edges) {
    parent[edge.second] = edge.first;    // edge.second의 부모는 edge.first
    child_count[edge.first]++;           // edge.first의 자식 수 증가
  }

  // ──────────────────────────────────────────────────────────
  //  [Step 9] Leaf 노드 수집
  // ──────────────────────────────────────────────────────────
  //  child_count가 0인 노드 = 더 이상 감염시킨 자식이 없는 말단 노드.
  //  각 leaf에서 root까지 역추적하면 하나의 경로가 된다.
  std::vector<int> leaves;
  for (int i = 0; i < cn; ++i) {
    if (child_count[i] == 0) leaves.push_back(i);
  }

  // ──────────────────────────────────────────────────────────
  //  [Step 10~12] 각 leaf → seed 역추적 → 리샘플링
  // ──────────────────────────────────────────────────────────
  std::vector<ChainedPoint> result;
  result.reserve(chain.size() * 2);   // 보간점이 추가되므로 넉넉하게 예약

  for (const int leaf : leaves) {
    // ── [Step 10] Leaf → Seed 역추적 ──
    //  parent를 따라 올라가며 경로의 인덱스를 수집한다.
    //  parent[root] == -1 이므로 루프가 종료된다.
    //  결과: path_idx = [leaf, ..., child1, seed] (역순)
    std::vector<int> path_idx;
    for (int cur = leaf; cur >= 0; cur = parent[cur]) {
      path_idx.push_back(cur);
    }

    // ── [Step 11] 역순 뒤집기 → seed→leaf 순서 ──
    //  역추적이므로 [leaf, ..., seed] 순서인데,
    //  리샘플링은 seed부터 leaf 방향으로 해야 하므로 뒤집는다.
    std::reverse(path_idx.begin(), path_idx.end());
    // path_idx = [seed(0), ..., leaf]  ← seed가 맨 앞

    // ── [Step 12] Polyline 리샘플링: 각 구간(edge)에 보간점 삽입 ──
    //
    //  경로의 연속 두 점(a, b) 사이에 ds 간격으로 점을 삽입한다.
    //
    //  예시: a ────────── b  (거리 = 0.5m, ds = 0.1m)
    //        a · · · · · b   (4개의 보간점이 삽입됨)
    //
    //  보간점의 좌표: a + t * (b - a),  t = d / len
    //  d는 ds, 2*ds, 3*ds, ... (< len)
    for (size_t s = 0; s < path_idx.size(); ++s) {
      // 원본 점을 먼저 추가
      result.push_back(chain[path_idx[s]]);

      // 다음 점이 있으면 그 사이에 보간점을 삽입
      if (s + 1 < path_idx.size()) {
        const auto & a = chain[path_idx[s]];       // 구간 시작점
        const auto & b = chain[path_idx[s + 1]];   // 구간 끝점
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double len = std::sqrt(dx * dx + dy * dy);  // 두 점 사이 거리

        // 구간 거리가 ds 이상이어야 보간점을 삽입할 의미가 있다.
        // (ds 미만이면 이미 충분히 촘촘하므로 삽입 불필요)
        if (len >= ds) {
          // ── Edge 타입 결정 ──
          //  양 끝점(a, b) 모두 CONE이면 → 보간점도 CONE 타입
          //  하나라도 LANE이면 → 보간점은 LANE 타입
          //
          //  ※ 왜 이렇게 하는가?
          //    콘-콘 구간은 물리적 장애물 경계이므로, 보간점도 CONE으로
          //    유지해야 planner가 해당 구간을 장애물 경계로 인식한다.
          //    콘-차선 또는 차선-차선 구간은 LANE으로 처리하여
          //    경로 생성 시 다른 가중치를 적용할 수 있게 한다.
          const PointType seg_type =
            (a.type == PointType::CONE && b.type == PointType::CONE)
              ? PointType::CONE : PointType::LANE;

          // ds 간격으로 보간점 삽입 (끝점 b는 다음 반복에서 추가됨)
          for (double d = ds; d < len; d += ds) {
            const double t = d / len;   // 보간 비율 (0~1)
            // 선형 보간: P = a + t * (b - a)
            result.push_back({a.x + t * dx, a.y + t * dy, seg_type});
          }
        }
      }
    }
  }

  return result;   // 원본 점 + 보간점이 합쳐진 최종 체인 반환
}

// ══════════════════════════════════════════════════════════════
//                    chain() — 메인 체이닝 함수
// ══════════════════════════════════════════════════════════════
//
//  이 함수는 LineChainer의 진입점이다.
//  외부에서는 이 함수만 호출하면 된다.
//
//  처리 흐름:
//    1. candidates에서 left seed / right seed를 선택
//    2. visited 배열을 하나 생성 (좌/우 공유!)
//    3. left seed로 chain_one_side() → 좌측 체인
//    4. right seed로 chain_one_side() → 우측 체인
//       (left에서 visited된 점은 right에서 자동 제외)
//    5. 결과를 ChainResult에 담아 반환
//
// ══════════════════════════════════════════════════════════════
ChainResult LineChainer::chain(
  const std::vector<ChainedPoint> & candidates,
  const PlanningParams & params)
{
  ChainResult result;
  const int n = static_cast<int>(candidates.size());

  // 점이 2개 미만이면 체이닝할 수 없으므로 빈 결과를 반환
  if (n < 2) return result;

  // ──────────────────────────────────────────────────────────
  //  [Step 1] Seed 선택
  // ──────────────────────────────────────────────────────────
  //
  //  ego(자차)의 위치는 좌표계의 원점 (0, 0)이다.
  //  ego 좌표계 규칙: +x = 전방, +y = 좌측, -y = 우측
  //
  //  Left seed:  y > 0 (좌측)인 점 중 ego에서 가장 가까운 점
  //  Right seed: y < 0 (우측)인 점 중 ego에서 가장 가까운 점
  //
  //  ※ 왜 ego에서 가장 가까운 점을 seed로 잡는가?
  //    ego 바로 옆에 있는 점이 해당 측면 경계에 속할 확률이 가장 높다.
  //    멀리 있는 점은 경로가 꺾이면서 반대쪽에 속할 수도 있으므로,
  //    가장 확실한 점(가장 가까운 점)에서 BFS를 시작하는 것이 안전하다.
  //
  //  ※ y == 0.0인 점은?
  //    좌/우 어느 쪽에도 확실히 속하지 않으므로 seed 후보에서 제외한다.
  //    (BFS 중 이웃으로 감염될 수는 있다)
  int left_seed = -1, right_seed = -1;
  double left_dist_sq = std::numeric_limits<double>::max();    // 좌측 최소 거리²
  double right_dist_sq = std::numeric_limits<double>::max();   // 우측 최소 거리²

  for (int i = 0; i < n; ++i) {
    // ego(0,0)까지의 제곱 거리 (sqrt 없이 비교)
    const double d_sq = candidates[i].x * candidates[i].x +
                        candidates[i].y * candidates[i].y;
    if (candidates[i].y > 0.0) {
      // y > 0: 좌측 점 → left seed 후보
      if (d_sq < left_dist_sq) {
        left_dist_sq = d_sq;
        left_seed = i;
      }
    } else if (candidates[i].y < 0.0) {
      // y < 0: 우측 점 → right seed 후보
      if (d_sq < right_dist_sq) {
        right_dist_sq = d_sq;
        right_seed = i;
      }
    }
    // y == 0.0인 점은 seed 선택에서 제외 (좌/우 판별 불가)
  }

  // 좌/우 seed 모두 없으면 체이닝 불가 → 빈 결과 반환
  if (left_seed < 0 && right_seed < 0) return result;

  // ──────────────────────────────────────────────────────────
  //  [Step 2] Visited 배열 생성 (좌/우 공유)
  // ──────────────────────────────────────────────────────────
  //
  //  ★ 핵심 설계: visited를 좌/우 양쪽이 공유한다.
  //
  //  왜 공유하는가?
  //    한 점이 좌측과 우측 경계에 동시에 속하면 안 된다.
  //    예를 들어, 좌측 체인의 BFS가 중앙 근처 점을 감염시켰는데,
  //    우측 체인의 BFS도 같은 점을 감염시키면 경계가 겹쳐버린다.
  //
  //    visited를 공유하면:
  //    - left의 chain_one_side()가 먼저 실행되어 좌측 점들을 visited=true로 세팅
  //    - right의 chain_one_side()는 이미 visited=true인 점을 건너뜀
  //    → 자연스럽게 좌/우 경계가 겹치지 않게 된다.
  //
  //  ※ left를 먼저 처리하므로, 경계가 모호한 중앙 점은 left에 우선 배정된다.
  std::vector<bool> visited(n, false);

  // ──────────────────────────────────────────────────────────
  //  [Step 3~4] 각 seed에서 Flood Fill 실행
  // ──────────────────────────────────────────────────────────
  std::vector<ChainedPoint> left_raw, right_raw;

  // 좌측 체인 구축 (left seed가 유효할 때만)
  // visited가 레퍼런스로 전달되므로, 이 호출 후 left에 소속된 점들은
  // visited[i] = true 상태가 된다.
  if (left_seed >= 0) {
    left_raw = chain_one_side(candidates, left_seed, visited, params);
  }

  // 우측 체인 구축 (right seed가 유효할 때만)
  // 위에서 left가 visited로 마킹한 점들은 여기서 자동으로 건너뛴다.
  if (right_seed >= 0) {
    right_raw = chain_one_side(candidates, right_seed, visited, params);
  }

  // ──────────────────────────────────────────────────────────
  //  [Step 5] 결과 조립
  // ──────────────────────────────────────────────────────────
  //  chain_one_side() 내부에서 이미 리샘플링(보간점 삽입)이 완료되었으므로,
  //  추가 후처리 없이 그대로 결과에 넣는다.
  //  std::move로 불필요한 복사를 방지한다.
  result.left_chain = std::move(left_raw);
  result.right_chain = std::move(right_raw);

  // 좌/우 체인 중 하나라도 비어있지 않으면 유효한 결과로 판정
  result.valid = (!result.left_chain.empty() || !result.right_chain.empty());
  return result;
}

}  // namespace planning_lc_ver
