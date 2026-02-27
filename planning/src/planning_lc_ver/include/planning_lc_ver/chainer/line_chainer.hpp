/**
 * @file line_chainer.hpp
 * @brief LineChainer — Flood fill 기반 경계점 체이닝
 *
 * ══════════════════════════════════════════════════════════════
 *                    LineChainer 전체 개요
 * ══════════════════════════════════════════════════════════════
 *
 * [목적]
 *   자율주행 차량(ego) 주변의 경계점(콘 + 차선)을 좌/우 경계선으로
 *   분류하여, 주행 가능한 corridor(복도)를 정의하는 것이다.
 *
 * [입력]
 *   - candidates: 콘(CONE)과 차선(LANE) 타입이 태그된 2D 점들의 집합
 *     (ego 좌표계 기준, 전방이 +x, 좌측이 +y)
 *
 * [출력]
 *   - ChainResult: 좌/우 경계 체인 (리샘플링 완료)
 *     - left_chain:  ego 좌측 경계를 구성하는 점열
 *     - right_chain: ego 우측 경계를 구성하는 점열
 *     - valid: 유효성 플래그
 *
 * ──────────────────────────────────────────────────────────────
 *                    알고리즘 전체 흐름
 * ──────────────────────────────────────────────────────────────
 *
 *   [Step 1] Seed 선택
 *     - y > 0 인 점 중 ego(0,0)에서 가장 가까운 점 → left seed
 *     - y < 0 인 점 중 ego(0,0)에서 가장 가까운 점 → right seed
 *     ※ ego에서 가장 가까운 점을 seed로 잡는 이유:
 *       가장 확실하게 해당 측면에 속하는 점이기 때문이다.
 *       멀리 있는 점은 반대쪽 경계에 속할 가능성이 있다.
 *
 *   [Step 2] Flood Fill (BFS 기반 영역 확장)
 *     - seed에서 BFS를 시작하여, 탐색 반경(search_radius) 이내의
 *       미방문 점들을 "감염"시키며 같은 체인에 소속시킨다.
 *     - BFS 큐에서 점을 하나 꺼내고, 그 주변 이웃을 찾아 큐에 넣는
 *       과정을 반복한다.
 *
 *     [Visited 공유]
 *       - left_chain과 right_chain이 하나의 visited 배열을 공유한다.
 *       - 이유: 한 점이 좌/우 양쪽에 동시에 소속되는 것을 방지하기 위함.
 *       - left를 먼저 처리하면, left에 소속된 점은 right 처리 시
 *         이미 visited=true이므로 건너뛴다.
 *
 *     [탐색 반경 확장]
 *       - 현재 반경(search_radius)에서 이웃이 없으면,
 *         search_radius_step만큼 반경을 늘려 재탐색한다.
 *       - search_radius_max까지 확장하며, 그래도 없으면 해당 노드의
 *         BFS는 종료된다 (더 이상 확장 불가).
 *       - 이웃을 찾으면 즉시 확장 루프를 탈출(break)한다.
 *
 *     [Cone Priority (콘 우선)]
 *       - 반경 내에 콘(CONE)과 차선(LANE) 점이 혼재하면,
 *         차선 점은 스킵하고 콘 점만 감염시킨다.
 *       - 이유: 콘은 물리적 장애물이므로 경로 경계로서 더 확실하다.
 *         차선 점은 콘 사이의 빈 구간을 보완하는 용도로만 사용한다.
 *       - 콘만 있으면 콘만, 차선만 있으면 차선만, 혼합이면 콘만 취한다.
 *
 *     [Edge(간선) 기록]
 *       - BFS 중 parent→child 관계를 edges 벡터에 저장한다.
 *       - 이 관계가 "감염 트리"를 형성한다: seed가 root이고,
 *         각 감염된 점이 자식 노드가 된다.
 *
 *   [Step 3] 트리 → 경로(Polyline) 추출
 *     - 감염 트리에서 child가 없는 노드(leaf)를 찾는다.
 *     - 각 leaf에서 parent를 따라 seed(root)까지 역추적한다.
 *     - 역추적 결과를 뒤집으면 seed→leaf 경로가 된다.
 *     - 하나의 seed에서 여러 가지(branch)가 뻗어나갈 수 있으므로,
 *       leaf 수만큼의 경로가 추출된다.
 *
 *   [Step 4] Edge별 리샘플링 (보간)
 *     - 각 경로의 연속 두 점(edge) 사이에 resample_ds 간격으로
 *       보간점(interpolation point)을 삽입한다.
 *     - 보간점의 타입(PointType):
 *       → 양 끝점 모두 CONE이면 CONE, 그 외에는 LANE
 *     - 이유: 콘-콘 구간은 장애물 경계이므로 CONE 타입을 유지해야
 *       하위 모듈(planner)이 해당 구간을 회피 경계로 인식할 수 있다.
 *
 * ──────────────────────────────────────────────────────────────
 *                    Flood Fill 개념도
 * ──────────────────────────────────────────────────────────────
 *
 *   ego(0,0) 기준:
 *
 *        +y (좌측)
 *         │
 *    L2 ──L1(seed)── ... ──Ln(leaf)
 *         │ \
 *   ──────┼──────── +x (전방)
 *         │
 *    R1(seed)── R2 ── ... ──Rm(leaf)
 *         │
 *        -y (우측)
 *
 *   L1에서 BFS → L2, L3, ... 감염 (좌측 체인)
 *   R1에서 BFS → R2, R3, ... 감염 (우측 체인)
 *   L1에 감염된 점은 R1의 BFS에서 건너뜀 (visited 공유)
 *
 * ══════════════════════════════════════════════════════════════
 */
#ifndef PLANNING_LC_VER__CHAINER__LINE_CHAINER_HPP_
#define PLANNING_LC_VER__CHAINER__LINE_CHAINER_HPP_

#include "planning_lc_ver/common/types.hpp"   // ChainedPoint, ChainResult, PointType 등
#include "planning_lc_ver/common/params.hpp"   // PlanningParams (search_radius 등 포함)

#include <vector>

namespace planning_lc_ver
{

/**
 * ────────────────────────────────────────────────────────────
 *  LineChainer 클래스
 * ────────────────────────────────────────────────────────────
 *
 * 이 클래스는 Flood Fill(BFS) 기반으로 경계점들을 좌/우 체인으로
 * 분류하는 역할을 한다.
 *
 * 사용법:
 *   LineChainer chainer;
 *   ChainResult result = chainer.chain(candidates, params);
 *   // result.left_chain  → 좌측 경계 점열 (리샘플링 완료)
 *   // result.right_chain → 우측 경계 점열 (리샘플링 완료)
 *   // result.valid       → 유효 여부
 *
 * 주요 파라미터 (PlanningParams::chainer 내부):
 *   - search_radius:      초기 BFS 탐색 반경 [m]
 *   - search_radius_max:  최대 확장 가능 반경 [m]
 *   - search_radius_step: 반경 확장 단계 [m]
 *   - resample_ds:        리샘플링 간격 [m] (edge 보간)
 * ────────────────────────────────────────────────────────────
 */
class LineChainer
{
public:
  /**
   * @brief 메인 체이닝 함수 — 모든 경계점을 좌/우 체인으로 분류한다
   *
   * 전체 처리 순서:
   *   1. candidates에서 left seed (y>0 최근접)와 right seed (y<0 최근접)를 선택
   *   2. visited 배열을 하나 생성하여 좌/우가 공유
   *   3. left seed로 chain_one_side() 호출 → 좌측 체인 생성
   *   4. right seed로 chain_one_side() 호출 → 우측 체인 생성
   *      (이때 left에서 이미 visited된 점은 right에서 자동 제외)
   *   5. ChainResult에 담아 반환
   *
   * @param candidates 모든 경계점 (콘+차선, PointType 태그 포함)
   *                   - ego 좌표계 기준: +x=전방, +y=좌측
   * @param params     체이너 파라미터 (search_radius, resample_ds 등)
   * @return ChainResult 좌/우 체인 (edge별 보간점 포함) + valid 플래그
   *
   * @note candidates가 2개 미만이면 빈 결과를 반환한다.
   * @note 좌/우 seed 모두 없으면 (모든 점이 y==0) 빈 결과를 반환한다.
   */
  ChainResult chain(
    const std::vector<ChainedPoint> & candidates,
    const PlanningParams & params);

private:
  /**
   * @brief 한쪽(좌 또는 우) 점 집합을 flood fill로 수집 + 트리 추출 + 리샘플링
   *
   * ── Flood Fill (BFS) 단계 ──
   *   1. seed를 visited로 마킹하고 chain에 추가, BFS 큐에 push
   *   2. 큐에서 pop한 점을 중심으로 탐색 반경 내 미방문 이웃 수집
   *   3. [Cone Priority] 이웃 중 콘+차선 혼합이면 차선 점 스킵
   *   4. [탐색 반경 확장] 이웃이 없으면 search_radius_step씩 반경 확장
   *      → search_radius_max까지 시도, 이웃 발견 즉시 확장 중단(break)
   *   5. 이웃을 visited 마킹 + chain에 추가 + 큐에 push
   *   6. parent→child edge를 기록 (감염 트리 구축)
   *   7. 큐가 빌 때까지 반복
   *
   * ── 트리 추출 + 리샘플링 단계 ──
   *   8. 감염 트리에서 leaf(자식 없는 노드)를 찾음
   *   9. 각 leaf에서 parent를 따라 seed까지 역추적 → 경로 추출
   *  10. 각 경로를 resample_ds 간격으로 리샘플링 (보간점 삽입)
   *      → 양끝 모두 CONE이면 보간점도 CONE, 아니면 LANE
   *
   * @param candidates 모든 후보점 (좌/우 공통 풀)
   * @param seed_idx   시작점 인덱스 (candidates 배열 내 인덱스)
   * @param visited    방문 배열 (좌/우 공유 — 레퍼런스로 전달)
   *                   → 이 함수 내에서 true로 설정한 점은 반대쪽에서 제외됨
   * @param params     파라미터 (search_radius, resample_ds 등)
   * @return 원본 점 + edge별 보간점이 합쳐진 체인
   *
   * @note static 함수: 인스턴스 상태를 사용하지 않으므로 정적 함수로 선언
   */
  static std::vector<ChainedPoint> chain_one_side(
    const std::vector<ChainedPoint> & candidates,
    int seed_idx,
    std::vector<bool> & visited,
    const PlanningParams & params);
};

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__CHAINER__LINE_CHAINER_HPP_
