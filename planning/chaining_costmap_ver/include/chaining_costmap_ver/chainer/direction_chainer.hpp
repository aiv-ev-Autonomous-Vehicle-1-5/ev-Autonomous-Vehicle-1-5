/**
 * @file direction_chainer.hpp
 * @brief DirectionChainer — Component → Backbone → Branch 기반 체이닝 (v2)
 *
 * ══════════════════════════════════════════════════════════════
 * [DirectionChainer란?]
 *
 *   LiDAR 콘(PE 드럼/교통 콘)과 카메라 차선 인식 경계점들을
 *   "좌측 경계"와 "우측 경계"로 분류하고,
 *   각 경계를 하나의 연속적인 라인(chain)으로 연결하는 모듈이다.
 *
 *   입력: ChainPoint[] (콘 + 차선 경계점, base_link 좌표)
 *   출력: DirectionChainResult (좌/우 SideResult)
 *         → costmap에 전달하여 경로 생성의 기반이 된다.
 *
 * ══════════════════════════════════════════════════════════════
 * [왜 6단계 파이프라인인가?]
 *
 *   경계점들은 불규칙하게 흩어져 있고, 노이즈/오탐이 섞여 있다.
 *   한 번에 연결하면 좌/우가 뒤섞이거나 지그재그 경로가 생긴다.
 *   따라서 단계적으로 필터→그래프→컴포넌트→주선→가지를 추출하여
 *   안정적인 경계 라인을 구성한다.
 *
 * ──────────────────────────────────────────────────────────────
 * [6단계 파이프라인 상세]
 *
 *   ┌─────────────────────────────────────────────────────────┐
 *   │ 1단계: Seed 선택 (Seed Selection)                       │
 *   │   - 좌/우 각각 시작점(seed) 선택                         │
 *   │   - side_seed_y 가드: 중심선 부근 점 제외 (|y| < 가드)   │
 *   │   - 선택 기준: ego(원점)에 가장 가까운 점                 │
 *   ├─────────────────────────────────────────────────────────┤
 *   │ 2단계: Undirected Graph 구성 (Build Graph)              │
 *   │   - 모든 점에 대해 kNN(k-최근접 이웃) 탐색               │
 *   │   - G1 거리 게이트: d(i,j) ≤ d_max 만 연결              │
 *   │   - G3 횡오차 게이트: |Δy| ≤ lateral_gate 만 연결        │
 *   │   - 결과: 양방향(undirected) 인접 리스트                  │
 *   ├─────────────────────────────────────────────────────────┤
 *   │ 3단계: Component 추출 (BFS from seed)                   │
 *   │   - seed에서 BFS → 연결된 모든 노드를 하나의 component로  │
 *   │   - visited 배열을 좌/우가 공유 → 한 점이 양쪽에 배정되지  │
 *   │     않게 보장 (ego에 가까운 seed 먼저 처리)               │
 *   ├─────────────────────────────────────────────────────────┤
 *   │ 4단계: Backbone 추출 (Greedy Chaining)                  │
 *   │   - component 내에서 seed부터 전방으로 탐욕적 확장        │
 *   │   - 비용함수 w' = w + λ·C_side 최소인 다음 노드 선택     │
 *   │     w = α·C_d + β·C_a + γ·C_lat + δ·C_size             │
 *   │   - 3개 게이트 (G1 거리, G2 전방 콘, G3 횡오차) 통과     │
 *   │     후보만 비용 계산 → 최소 비용 노드로 이동              │
 *   │   - 종료 조건: 후보 소진 / 모두 게이트 탈락 / max_len    │
 *   ├─────────────────────────────────────────────────────────┤
 *   │ 5단계: Branch 추출 (Residual → Backbone 연결)           │
 *   │   - backbone에 포함되지 않은 잔여 노드들                  │
 *   │   - 각 잔여 노드에서 BFS → 가장 가까운 backbone 노드 탐색 │
 *   │   - 같은 backbone 노드에 연결된 잔여 노드들을 묶어 branch │
 *   │   - branch = 콘/차선이 backbone 옆으로 갈라진 가지        │
 *   ├─────────────────────────────────────────────────────────┤
 *   │ 6단계: Component 리샘플링 (Resample)                    │
 *   │   - backbone + branch의 모든 edge를 일정 간격으로 보간    │
 *   │   - resample_ds 간격으로 균등한 점열 생성                 │
 *   │   - costmap에 전달할 최종 경계점열 완성                   │
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
 *   │ C_size │ 크기 변화: |size_j-size_i|/size_i (콘 전용)   │
 *   │        │ → 비슷한 크기의 콘끼리 연결 유도               │
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
 *   G2 (전방 콘 게이트): angle(v_i, u_ij) ≤ forward_cone_deg/2
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

#include <vector>

namespace chaining_costmap_ver
{

/**
 * @class DirectionChainer
 * @brief 6단계 파이프라인으로 경계점을 좌/우 체인으로 분류하는 핵심 클래스
 *
 * [사용 방법]
 *   DirectionChainer chainer;
 *   auto result = chainer.chain(all_boundary_points, params);
 *   // result.left.component → 좌측 경계점열 (costmap 전달용)
 *   // result.right.component → 우측 경계점열 (costmap 전달용)
 *
 * [설계 철학]
 *   - 모든 상태를 멤버로 보관하지 않음 (stateless)
 *   - chain() 호출마다 0~6단계를 순차 실행
 *   - const 메서드로 스레드 안전
 */
class DirectionChainer
{
public:
  /**
   * @brief 메인 진입점 — 6단계 파이프라인 전체 실행
   *
   * 모든 경계점(콘+차선)을 입력받아 좌/우로 분류하고,
   * 각 방향의 backbone(주선) + branch(가지)를 추출하여
   * 리샘플링된 경계점열을 반환한다.
   *
   * @param points  모든 경계점 (ChainPoint, 콘+차선 통합)
   *                - LiDAR DBSCAN 결과의 콘 중심점
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
   *   3. ego(원점)에서 가장 가까운 점을 seed로 선택 (콘/차선 구분 없이)
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
  // 2단계: Undirected Graph 구성 — kNN + 게이트 필터
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
   *     → 2단계에서는 방향 정보가 없으므로 단순 y 차이로 판정한다.
   *   ※ G2 (전방 콘 게이트)는 여기서 적용하지 않는다.
   *     → 방향 벡터가 없기 때문. 4단계 backbone에서 동적으로 적용된다.
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
  // 3단계: Component 추출 — BFS로 연결 성분 분리
  // ═══════════════════════════════════════════════════════════
  /**
   * @brief seed에서 BFS로 연결된 모든 노드를 하나의 component로 추출
   *
   * [왜 visited를 공유하는가?]
   *   - 좌측 seed와 우측 seed가 각각 BFS를 실행한다.
   *   - visited 배열을 공유하면, 먼저 처리된 쪽이 차지한 노드는
   *     반대쪽에서 접근할 수 없다.
   *   - 이렇게 하면 하나의 점이 좌/우 양쪽에 배정되는 것을 방지한다.
   *   - ego에 가까운 seed를 먼저 처리하여 더 확실한 쪽이 우선권을 가진다.
   *
   * @param graph     2단계에서 구성한 undirected 그래프
   * @param seed_idx  BFS 시작점 (seed)
   * @param visited   [in/out] 방문 표시 배열 (좌/우 공유)
   * @return component에 포함된 점 인덱스 배열
   */
  std::vector<int> extract_component(
    const ChainingGraph & graph,
    int seed_idx,
    std::vector<bool> & visited) const;

  // ═══════════════════════════════════════════════════════════
  // 4단계: Backbone 추출 — 탐욕적 체이닝으로 주선 추출
  // ═══════════════════════════════════════════════════════════
  /**
   * @brief component 내에서 seed→전방으로 greedy하게 주 경계선 추출
   *
   * [Greedy Chaining 알고리즘]
   *   1. seed를 시작점으로, 초기 진행 방향 v = (1,0) (전방)
   *   2. 현재 노드의 kNN 중 component 내 노드만 필터
   *   3. 3개 게이트 적용:
   *      - G1: d(cur, j) ≤ d_max (거리)
   *      - G2: angle(v, u_ij) ≤ cone_half (전방 콘)
   *      - G3: |lateral_proj| ≤ lateral_gate (횡오차)
   *   4. 게이트 통과한 후보들에 대해 w' 비용 계산
   *   5. w' 최소인 노드를 다음 노드로 선택, 진행 방향 갱신
   *   6. 반복 (후보 소진 / 모두 게이트 탈락 / max_chain_len 도달 시 종료)
   *
   * [왜 Greedy인가?]
   *   - 전역 최적해(예: 최단 경로)는 O(n!) 이상 걸릴 수 있다.
   *   - 경계점은 물리적으로 순서가 있으므로, 가장 자연스러운 다음 점을
   *     선택하는 greedy 방식이 실시간에 적합하고 결과도 충분히 좋다.
   *
   * @param points         필터링된 경계점 배열
   * @param component_ids  3단계에서 추출한 component 내 인덱스들
   * @param seed_idx       시작점 인덱스
   * @param is_left        좌측(true) / 우측(false) — C_side 계산에 사용
   * @param stop_reason    [out] 체이닝이 왜 멈췄는지 기록
   * @param cp             chainer 파라미터
   * @return backbone 노드 인덱스 배열 (seed → goal 순서)
   */
  std::vector<int> extract_backbone(
    const std::vector<ChainPoint> & points,
    const std::vector<int> & component_ids,
    int seed_idx,
    bool is_left,
    StopReason & stop_reason,
    const PlanningParams::Chainer & cp) const;

  // ═══════════════════════════════════════════════════════════
  // 5단계: Branch 추출 — 잔여 노드를 backbone에 연결
  // ═══════════════════════════════════════════════════════════
  /**
   * @brief backbone에 포함되지 않은 잔여 노드를 가지(branch)로 구성
   *
   * [Branch란?]
   *   backbone은 component의 "주 경계선"이다.
   *   하지만 component에는 backbone에 포함되지 못한 점들이 있다.
   *   예: backbone 옆에 있는 콘, 곡선 구간에서 빠진 차선점 등.
   *   이런 점들을 무시하면 costmap에 빈 영역이 생긴다.
   *   branch로 연결하면 경계 정보를 최대한 활용할 수 있다.
   *
   * [알고리즘]
   *   1. component 중 backbone에 없는 잔여 노드 수집
   *   2. 각 잔여 노드에서 undirected 그래프 BFS 실행
   *   3. BFS 중 backbone 노드를 만나면 → 그 backbone 노드가 "부모"
   *   4. 같은 부모를 공유하는 잔여 노드들을 묶어 하나의 branch 구성
   *   5. 부모로부터 거리 순 정렬, max_branch_len으로 길이 제한
   *
   * @param graph          2단계의 undirected 그래프
   * @param points         필터링된 경계점 배열
   * @param component_ids  component 내 인덱스들
   * @param backbone_ids   4단계의 backbone 인덱스들
   * @param cp             chainer 파라미터 (max_branch_len 사용)
   * @return branch 정보 배열 (BranchInfo: 부모 위치, 점들, 스코어)
   */
  std::vector<BranchInfo> extract_branches(
    const ChainingGraph & graph,
    const std::vector<ChainPoint> & points,
    const std::vector<int> & component_ids,
    const std::vector<int> & backbone_ids,
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
   *   4. 보간점의 type: 양끝이 모두 CONE이면 CONE, 아니면 LANE
   *   5. 보간점의 confidence: 양끝 중 낮은 값 사용 (보수적)
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
   *   C_size: 크기 변화 비용 — 비슷한 크기의 콘끼리 연결 선호
   *           (콘↔콘일 때만 적용, 상대적 크기 변화율)
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
