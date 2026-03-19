/**
 * @file direction_chainer.hpp
 * @brief DirectionChainer — Owner-Label 기반 순차 체이닝 (v3)
 *
 * ══════════════════════════════════════════════════════════════
 * [DirectionChainer란?]
 *
 *   LiDAR bbox(PE 드럼)와 카메라 차선 인식 경계점들을
 *   "좌측 경계"와 "우측 경계"로 분류하고,
 *   각 경계를 하나의 연속적인 라인(chain)으로 연결하는 모듈이다.
 *
 *   입력: ChainPoint[] (bbox + 차선 경계점, base_link 좌표)
 *   출력: DirectionChainResult (좌/우 SideResult)
 *         → costmap에 전달하여 경로 생성의 기반이 된다.
 *
 * ══════════════════════════════════════════════════════════════
 * [왜 Owner-Label 기반 파이프라인인가?]
 *
 *   v2에서는 visited 배열을 좌/우 BFS가 공유하여 component를 분리했다.
 *   하지만 bridge 노드(y≈0 근처)를 통해 한쪽이 반대편 노드를 선점하는
 *   문제가 있었다. v3에서는 NodeOwner 라벨로 노드 소유권을 관리하여
 *   이 문제를 해결한다.
 *
 * ──────────────────────────────────────────────────────────────
 * [5단계 파이프라인 상세]
 *
 *   ┌─────────────────────────────────────────────────────────┐
 *   │ 준비: Seed 선택 + Undirected Graph 구성                  │
 *   │   - 좌/우 각각 시작점(seed) 선택                         │
 *   │   - 모든 점에 대해 kNN + G1,G3 게이트로 그래프 구성       │
 *   │   - owner 배열 초기화 (all NONE)                         │
 *   ├─────────────────────────────────────────────────────────┤
 *   │ 1단계: Left Backbone 확정 (전방 전용)                     │
 *   │   - left seed에서 forward(+x) 방향으로만 chaining       │
 *   │   - owner==NONE만 후보                                  │
 *   │   - 결과: [seed] + forward → 최종 backbone              │
 *   │   - 확정된 노드에 LEFT_BACKBONE 라벨 부여                │
 *   ├─────────────────────────────────────────────────────────┤
 *   │ 2단계: Right Backbone 확정 (전방 전용)                   │
 *   │   - right seed에서 forward(+x) 방향으로만 chaining      │
 *   │   - owner==NONE만 후보, LEFT_BACKBONE 노드는 자동 제외   │
 *   │   - 확정된 노드에 RIGHT_BACKBONE 라벨 부여               │
 *   ├─────────────────────────────────────────────────────────┤
 *   │ 3단계: Left Branch 확정                                 │
 *   │   - left backbone 노드를 순서대로 순회하며 BFS            │
 *   │   - 허용: NONE, LEFT_BRANCH / 차단: 모든 BACKBONE        │
 *   │   - 같은 side branch 중복 소속 허용 (촘촘한 costmap 장벽) │
 *   │   - 확정된 노드에 LEFT_BRANCH 라벨 부여                  │
 *   ├─────────────────────────────────────────────────────────┤
 *   │ 4단계: Right Branch 확정                                │
 *   │   - right backbone 노드를 순서대로 순회하며 BFS           │
 *   │   - 허용: NONE, RIGHT_BRANCH                             │
 *   │   - 차단: LEFT_BACKBONE, RIGHT_BACKBONE, LEFT_BRANCH     │
 *   │   - 확정된 노드에 RIGHT_BRANCH 라벨 부여                 │
 *   ├─────────────────────────────────────────────────────────┤
 *   │ 5단계: Resample                                         │
 *   │   - 좌/우 각각 backbone + branch의 모든 edge를 보간       │
 *   │   - resample_ds 간격으로 균등한 점열 생성                 │
 *   └─────────────────────────────────────────────────────────┘
 *
 * ──────────────────────────────────────────────────────────────
 * [비용함수 상세]
 *
 *   기본 비용 w(i,j):
 *     w = α·C_d + β·C_a + γ·C_lat + δ·C_size
 *
 *   확장 비용 w'(i,j):
 *     w' = w + λ_side · C_side
 *
 *   ┌────────┬──────────────────────────────────────────────┐
 *   │ 비용   │ 물리적 의미                                   │
 *   ├────────┼──────────────────────────────────────────────┤
 *   │ C_d    │ 거리 비용: d(i,j)/d_max                      │
 *   │        │ → 가까운 점을 선호 (짧은 연결 유도)            │
 *   ├────────┼──────────────────────────────────────────────┤
 *   │ C_a    │ 방향 오차: angle(v_i, u_ij)/θ_max            │
 *   │        │ → 현재 진행 방향과 같은 방향의 점 선호          │
 *   │        │ → 지그재그 방지, 부드러운 체인 유도             │
 *   ├────────┼──────────────────────────────────────────────┤
 *   │ C_lat  │ 횡오차: |lateral_projection|/lateral_gate     │
 *   │        │ → 진행 방향 수직 성분이 작은 점 선호            │
 *   │        │ → 체인이 옆으로 튀는 것 방지                   │
 *   ├────────┼──────────────────────────────────────────────┤
 *   │ C_size │ 크기 변화: |size_j-size_i|/size_i (bbox 전용)  │
 *   │        │ → 비슷한 크기의 bbox끼리 연결 유도             │
 *   │        │ → 다른 크기의 장애물과 혼동 방지                │
 *   ├────────┼──────────────────────────────────────────────┤
 *   │ C_side │ 측면 선호도: 중심선 쪽으로 이동 시 페널티       │
 *   │        │ → 좌측: y 감소(안쪽) 시 페널티                 │
 *   │        │ → 우측: y 증가(안쪽) 시 페널티                 │
 *   │        │ → 체인이 반대편으로 넘어가는 것 방지            │
 *   └────────┴──────────────────────────────────────────────┘
 *
 * ──────────────────────────────────────────────────────────────
 * [게이트(Gate) 상세]
 *
 *   G1 (거리 게이트):  d(i,j) ≤ d_max
 *     → 너무 먼 점은 같은 경계가 아님 (연결 차단)
 *
 *   G2 (전방 cone 게이트): angle(v_i, u_ij) ≤ forward_cone_deg/2
 *     → 뒤쪽/옆쪽 점은 "다음 경계점"이 아님
 *     → backbone에서만 적용 (graph 구성 시에는 미적용)
 *
 *   G3 (횡오차 게이트): |Δy| ≤ lateral_gate  (그래프용)
 *                       |lat_proj| ≤ lateral_gate  (backbone용)
 *     → 횡방향으로 너무 떨어진 점 연결 차단
 *     → 좌/우 경계가 서로 연결되는 것 방지
 *
 * ══════════════════════════════════════════════════════════════
 */
#ifndef CHAINING_COSTMAP_VER__CHAINER__DIRECTION_CHAINER_HPP_
#define CHAINING_COSTMAP_VER__CHAINER__DIRECTION_CHAINER_HPP_

#include "chaining_costmap_ver/common/types.hpp"
#include "chaining_costmap_ver/common/params.hpp"

#include <unordered_set>
#include <vector>

namespace chaining_costmap_ver
{

/**
 * @class DirectionChainer
 * @brief Owner-Label 기반 파이프라인으로 경계점을 좌/우 체인으로 분류하는 핵심 클래스
 *
 * [사용 방법]
 *   DirectionChainer chainer;
 *   auto result = chainer.chain(all_boundary_points, params);
 *   // result.left.component → 좌측 경계점열 (costmap 전달용)
 *   // result.right.component → 우측 경계점열 (costmap 전달용)
 *
 * [설계 철학]
 *   - 모든 상태를 멤버로 보관하지 않음 (stateless)
 *   - chain() 호출마다 seed→graph→backbone→branch→resample 순차 실행
 *   - const 메서드로 스레드 안전
 */
class DirectionChainer
{
public:
  /**
   * @brief 메인 진입점 — 6단계 파이프라인 전체 실행
   *
   * 모든 경계점(bbox+차선)을 입력받아 좌/우로 분류하고,
   * 각 방향의 backbone(주선) + branch(가지)를 추출하여
   * 리샘플링된 경계점열을 반환한다.
   *
   * @param points  모든 경계점 (ChainPoint, bbox+차선 통합)
   *                - LiDAR DBSCAN 결과의 bbox 중심점
   *                - 카메라 차선 인식 결과의 경계점
   * @param params  전체 파라미터 (PlanningParams)
   *                - params.chainer 섹션의 값들이 사용됨
   * @return DirectionChainResult
   *   - left: 좌측 SideResult (component, backbone, branches)
   *   - right: 우측 SideResult
   *   - valid: 최소 한쪽 backbone이 생성되었는지 여부
   */
  DirectionChainResult chain(
    const std::vector<ChainPoint> & points,
    const PlanningParams & params) const;

private:
  // ═══════════════════════════════════════════════════════════
  // 1단계: Seed 선택 — 체이닝 시작점 결정
  // ═══════════════════════════════════════════════════════════
  /**
   * @brief 좌측 또는 우측의 체이닝 시작점(seed) 선택
   *
   * [알고리즘]
   *   1. x ≥ -2.0 필터: 후방 2m 이상인 점은 seed 후보에서 제외
   *   2. side_seed_y 가드:
   *      - 좌측(is_left=true):  y ≥ side_seed_y 인 점만 후보
   *      - 우측(is_left=false): y ≤ -side_seed_y 인 점만 후보
   *      → 중심선(y≈0) 근처 점이 잘못된 방향의 seed가 되는 것 방지
   *   3. ego(원점)에서 가장 가까운 점을 seed로 선택 (bbox/차선 구분 없이)
   *
   * @param points   필터링된 경계점 배열
   * @param is_left  true=좌측 seed, false=우측 seed
   * @param cp       chainer 파라미터 (side_seed_y 사용)
   * @return seed 인덱스 (후보 없으면 -1)
   */
  int find_seed(
    const std::vector<ChainPoint> & points,
    bool is_left,
    const PlanningParams::Chainer & cp) const;

  // ═══════════════════════════════════════════════════════════
  // Undirected Graph 구성 — kNN + 게이트 필터
  // ═══════════════════════════════════════════════════════════
  /**
   * @brief 모든 점 간의 연결 관계를 kNN + 게이트로 구성
   *
   * [왜 kNN을 사용하는가?]
   *   - 모든 점 쌍(O(n²))을 검사하면 비효율적이다.
   *   - 각 점의 k개 최근접 이웃만 검사하면 O(n·k)로 충분하다.
   *   - 경계점은 보통 수십~수백 개이므로 brute-force kNN으로 충분하다.
   *
   * [게이트의 역할]
   *   - G1 (거리 게이트): d(i,j) ≤ d_max
   *     → 너무 먼 점끼리 연결되면 다른 경계의 점이 섞일 수 있다.
   *   - G3 (횡오차 게이트): |Δy| ≤ lateral_gate
   *     → 좌/우 경계가 그래프에서 직접 연결되는 것을 방지한다.
   *     → 방향 정보가 없으므로 단순 y 차이로 판정한다.
   *   ※ G2 (전방 cone 게이트)는 여기서 적용하지 않는다.
   *     → 방향 벡터가 없기 때문. backbone에서 동적으로 적용된다.
   *
   * [출력]
   *   - ChainingGraph.undirected: 양방향 인접 리스트
   *   - undirected[i] = {j1, j2, ...} → i와 연결된 이웃 인덱스들
   *
   * @param points  필터링된 경계점 배열
   * @param cp      chainer 파라미터 (k, d_max, lateral_gate 사용)
   * @return ChainingGraph  양방향 인접 리스트 그래프
   */
  ChainingGraph build_graph(
    const std::vector<ChainPoint> & points,
    const PlanningParams::Chainer & cp) const;

  // ═══════════════════════════════════════════════════════════
  // 1-2단계: Backbone 추출 — 전방 전용 탐욕적 체이닝으로 주선 추출
  // ═══════════════════════════════════════════════════════════
  /**
   * @brief seed에서 전방(+x)으로만 greedy하게 주 경계선 추출
   *
   * [전방 전용 Greedy Chaining 알고리즘]
   *   1) Forward pass: seed에서 v={1,0} 방향으로 greedy chaining
   *   2) 결과: [seed] + forward → 최종 backbone
   *
   *   [결과]
   *     seed            forward 끝
   *     [S] → ● → ● → ● → ●
   *
   * [게이트 적용]
   *   - G1: d(cur, j) ≤ d_max (거리)
   *   - G2: angle(v, u_ij) ≤ cone_half (전방 cone)
   *   - G3: |lateral_proj| ≤ lateral_gate (횡오차)
   *
   * [BBOX 우선 선택]
   *   게이트를 통과한 후보를 BBOX와 LANE으로 분리한 뒤,
   *   bbox 후보가 존재하면 bbox만으로 w' 최소 비용 선택을 수행한다.
   *   bbox가 없을 때만 lane 후보로 fallback한다.
   *
   * [owner 기반 필터링]
   *   owner[j] == NONE인 노드만 후보로 허용하므로,
   *   이미 LEFT_BACKBONE으로 확정된 노드는 right backbone 후보에서 자동 제외된다.
   *
   * @param points              필터링된 경계점 배열
   * @param owner               [in] 각 노드의 소유권 라벨 배열
   * @param seed_idx            시작점 인덱스
   * @param is_left             좌측(true) / 우측(false) — C_side 계산에 사용
   * @param stop_reason_forward [out] 전방 체이닝 종료 이유
   * @param cp                  chainer 파라미터
   * @return backbone 노드 인덱스 배열 (seed → forward 끝 순서)
   */
  std::vector<int> extract_backbone(
    const std::vector<ChainPoint> & points,
    const std::vector<NodeOwner> & owner,
    int seed_idx,
    bool is_left,
    StopReason & stop_reason_forward,
    int & seed_backbone_pos,
    const PlanningParams::Chainer & cp) const;

  /**
   * @brief 단방향 greedy chaining 내부 헬퍼 (extract_backbone에서 호출)
   *
   * seed에서 주어진 초기 방향(init_dir)으로 한 방향만 체이닝한다.
   * extract_backbone()이 forward(+x) 방향에 대해 이 함수를 호출한다.
   *
   * [BBOX 우선 선택]
   *   각 스텝에서 게이트 통과 후보를 bbox/lane으로 분리 →
   *   bbox가 있으면 bbox만으로, 없으면 lane으로 w' 최소 비용 선택.
   *
   * @param points       경계점 배열
   * @param owner        [in] 소유권 라벨 배열
   * @param seed_idx     시작점 인덱스
   * @param init_dir     초기 진행 방향 단위벡터 (전방: {1,0}, 후방: {-1,0})
   * @param is_left      좌측/우측 — C_side 계산용
   * @param visited_set  [in/out] 방문 노드 set
   * @param remaining_len 남은 허용 체인 길이 (max_chain_len 합산 제한용)
   * @param stop_reason  [out] 체이닝 종료 이유
   * @param cp           chainer 파라미터
   * @return 체이닝된 노드 인덱스 배열 (seed 미포함, 진행 방향 순서)
   */
  std::vector<int> chain_one_direction(
    const std::vector<ChainPoint> & points,
    const std::vector<NodeOwner> & owner,
    int seed_idx,
    const Point2D & init_dir,
    bool is_left,
    std::unordered_set<int> & visited_set,
    int remaining_len,
    StopReason & stop_reason,
    const PlanningParams::Chainer & cp) const;

  // ═══════════════════════════════════════════════════════════
  // 3-4단계: Branch 추출 — Backbone 순회 기반 BFS
  // ═══════════════════════════════════════════════════════════
  /**
   * @brief backbone 노드를 순서대로 순회하며 주변 노드를 BFS로 branch에 연결
   *
   * [Branch란?]
   *   backbone 옆에 있지만 backbone에 선택되지 못한 bbox/차선점.
   *   이런 점들을 branch로 연결하면 costmap에 빈틈 없는 비용 장벽이 형성된다.
   *
   * [알고리즘 — Backbone 순회 기반 Greedy Chaining]
   *   backbone 노드를 B0→B1→B2→... 순서로 순회하며:
   *   1. Bi에서 d_max 범위 내의 허용 노드 중 가장 가까운 노드를 chain 시작점으로 선택
   *   2. chain 끝점에서 d_max 범위 내의 가장 가까운 미방문 허용 노드로 greedy 이동
   *   3. 같은 side의 다른 backbone에 이미 소속된 branch도 중복 연결 허용
   *   4. max_branch_len 제한, 모든 edge가 d_max 이내로 보장
   *
   * [허용 조건]
   *   owner[node] == NONE || owner[node] == branch_label
   *   → 같은 side의 branch 노드는 중복 소속 가능 (greedy 통과 + 재연결)
   *   → 반대 side의 backbone/branch는 차단
   *
   * @param points         필터링된 경계점 배열
   * @param backbone_ids   backbone 인덱스 배열
   * @param owner          [in/out] 소유권 라벨 (새 branch에 branch_label 부여)
   * @param branch_label   이 side의 branch 라벨 (LEFT_BRANCH 또는 RIGHT_BRANCH)
   * @param cp             chainer 파라미터 (max_branch_len 사용)
   * @return branch 정보 배열 (BranchInfo: 부모 위치, 점들, 스코어)
   */
  std::vector<BranchInfo> extract_branches(
    const std::vector<ChainPoint> & points,
    const std::vector<int> & backbone_ids,
    std::vector<NodeOwner> & owner,
    NodeOwner branch_label,
    const PlanningParams::Chainer & cp) const;

  // ═══════════════════════════════════════════════════════════
  // 6단계: Component 리샘플링 — 균등 간격 보간
  // ═══════════════════════════════════════════════════════════
  /**
   * @brief backbone + branch의 모든 edge를 resample_ds 간격으로 보간
   *
   * [왜 리샘플링이 필요한가?]
   *   - 원본 점들은 불균등한 간격으로 분포한다.
   *   - costmap에서 Gaussian 코스트를 그릴 때, 점 간격이 넓으면
   *     경계선에 "구멍"이 생겨 차량이 잘못된 경로를 탐색할 수 있다.
   *   - 균등 간격으로 보간하면 costmap에 연속적인 장벽이 형성된다.
   *
   * [리샘플 과정]
   *   1. backbone의 연속된 점 쌍(edge)을 순회
   *   2. 각 edge를 resample_ds 간격으로 선형 보간
   *   3. branch도 동일하게: 부모→첫점 edge + 내부 edge 모두 보간
   *   4. 보간점의 type: 양끝 모두 BBOX일 때만 BBOX, 혼합 edge는 LANE
   *
   * @param points       필터링된 경계점 배열
   * @param backbone_ids backbone 인덱스 배열
   * @param branches     branch 정보 배열
   * @param resample_ds  보간 간격 [m]
   * @return 리샘플링된 경계점 배열 (costmap 전달용)
   */
  std::vector<ChainPoint> resample_component(
    const std::vector<ChainPoint> & points,
    const std::vector<int> & backbone_ids,
    const std::vector<BranchInfo> & branches,
    double resample_ds) const;

  // ═══════════════════════════════════════════════════════════
  // 유틸리티 함수들
  // ═══════════════════════════════════════════════════════════

  /**
   * @brief k-최근접 이웃 탐색 (brute-force 방식)
   *
   * [왜 brute-force인가?]
   *   - 경계점 수가 보통 수십~수백 개로 적다.
   *   - KD-Tree 등의 자료구조 오버헤드가 탐색 비용보다 클 수 있다.
   *   - partial_sort로 상위 k개만 정렬 → O(n·log(k))로 충분히 빠르다.
   *
   * @param points     전체 점 배열
   * @param query_idx  쿼리 점의 인덱스
   * @param k          찾을 이웃 수
   * @return k개 최근접 이웃의 인덱스 배열 (거리 오름차순)
   */
  std::vector<int> knn(
    const std::vector<ChainPoint> & points,
    int query_idx,
    int k) const;

  /**
   * @brief 기본 비용함수 w(i, j) 계산
   *
   * w = α·C_d + β·C_a + γ·C_lat + δ·C_size
   *
   * [각 비용 항의 물리적 의미]
   *   C_d   : 거리 비용 — 가까운 점 선호 (d/d_max, 0~1 정규화)
   *   C_a   : 방향 오차 비용 — 현재 진행 방향과 일치하는 점 선호
   *           (angle/θ_max, 0~1+ 정규화)
   *   C_lat : 횡오차 비용 — 진행 방향 수직 성분이 작은 점 선호
   *           (lat_offset/lateral_gate, 0~1 정규화)
   *   C_size: 크기 변화 비용 — 비슷한 크기의 bbox끼리 연결 선호
   *           (bbox↔bbox일 때만 적용, 상대적 크기 변화율)
   *
   * @param pi    현재 점
   * @param pj    후보 다음 점
   * @param v_i   현재 진행 방향 단위벡터
   * @param cp    chainer 파라미터 (alpha, beta, gamma, delta 가중치)
   * @return 비용 값 w (작을수록 좋은 연결)
   */
  double compute_cost(
    const ChainPoint & pi,
    const ChainPoint & pj,
    const Point2D & v_i,
    const PlanningParams::Chainer & cp) const;

  /**
   * @brief 확장 비용함수 w'(i, j) 계산 — side preference 포함
   *
   * w' = w(i,j) + λ_side · C_side
   *
   * [C_side의 역할]
   *   체인이 중심선 쪽으로 이동하면 페널티를 부여한다.
   *   - 좌측 경계: y가 감소(=안쪽으로 이동)하면 C_side > 0
   *   - 우측 경계: y가 증가(=안쪽으로 이동)하면 C_side > 0
   *   → 체인이 반대편 경계로 넘어가는 것을 방지한다.
   *   → λ_side가 클수록 좌/우 분리가 강해진다.
   *
   * @param pi       현재 점
   * @param pj       후보 다음 점
   * @param v_i      현재 진행 방향 단위벡터
   * @param is_left  좌측 경계(true) / 우측 경계(false)
   * @param cp       chainer 파라미터 (lambda_side 사용)
   * @return 확장 비용 값 w' (작을수록 좋은 연결)
   */
  double compute_cost_prime(
    const ChainPoint & pi,
    const ChainPoint & pj,
    const Point2D & v_i,
    bool is_left,
    const PlanningParams::Chainer & cp) const;
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__CHAINER__DIRECTION_CHAINER_HPP_
