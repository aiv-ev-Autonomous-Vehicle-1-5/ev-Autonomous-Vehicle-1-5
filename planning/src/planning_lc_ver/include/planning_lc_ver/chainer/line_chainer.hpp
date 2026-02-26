/**
 * @file line_chainer.hpp
 * @brief LineChainer — DFS 기반 nearest-neighbor 경계점 체이닝
 *
 * ──────────────────────────────────────────────────────────────
 * 역할: 콘/차선 점들을 좌/우 경계 체인으로 분류하고 리샘플링
 *
 * 알고리즘 흐름:
 *   1. Seed 선택: y>0인 점 중 ego 최근접 → left seed
 *                  y<0인 점 중 ego 최근접 → right seed
 *   2. DFS 체이닝: 각 seed에서 출발하여 방향벡터 기반 전진 탐색
 *      - visited 배열을 좌/우 공유 → 중복 소속 방지
 *      - 후보가 없으면 search_radius 확장 (max까지)
 *      - Cone priority: 콘 후보가 있으면 차선 제거
 *      - Scoring: cross2 기반으로 도로 안쪽 선호
 *   3. 노이즈 할당: DFS 후 미방문 점들을 가장 가까운 chain 점에 병렬 할당
 *   4. 리샘플링: 0.1m 간격, cone-cone 구간은 CONE 타입 유지
 * ──────────────────────────────────────────────────────────────
 */
#ifndef PLANNING_LC_VER__CHAINER__LINE_CHAINER_HPP_
#define PLANNING_LC_VER__CHAINER__LINE_CHAINER_HPP_

#include "planning_lc_ver/common/types.hpp"
#include "planning_lc_ver/common/params.hpp"

#include <vector>

namespace planning_lc_ver
{

class LineChainer
{
public:
  /**
   * @brief 모든 경계점을 좌/우 체인으로 분류하고 리샘플링
   *
   * @param candidates 모든 경계점 (콘+차선, 타입 태그 포함)
   * @param params     체이너 파라미터 (search_radius, forward_angle 등)
   * @return ChainResult 좌/우 리샘플된 체인 + valid 플래그
   */
  ChainResult chain(
    const std::vector<ChainedPoint> & candidates,
    const PlanningParams & params);

private:
  /**
   * @brief 한쪽(좌 또는 우) 체인을 DFS로 구축
   *
   * seed에서 출발하여 방향벡터 기반으로 다음 점을 탐색한다.
   * visited 배열은 좌/우 chain이 공유하여 중복 소속을 방지한다.
   *
   * @param candidates 모든 후보점
   * @param seed_idx   시작점 인덱스
   * @param is_left    true면 좌측 체인 (scoring 부호 반전)
   * @param visited    방문 배열 (공유)
   * @param params     파라미터
   * @return 체이닝된 점 목록 (리샘플 전)
   */
  static std::vector<ChainedPoint> chain_one_side(
    const std::vector<ChainedPoint> & candidates,
    int seed_idx,
    bool is_left,
    std::vector<bool> & visited,
    const PlanningParams & params);

  /**
   * @brief 체인을 ds 간격으로 리샘플링 (타입 보존)
   *
   * - cone-cone 구간의 보간점 → CONE 타입
   * - 그 외 (lane-lane, lane-cone) → LANE 타입
   *
   * @param chain_pts 원본 체인 점 목록
   * @param ds        리샘플 간격 [m]
   * @return 리샘플된 체인
   */
  static std::vector<ChainedPoint> resample_chain(
    const std::vector<ChainedPoint> & chain_pts,
    double ds);
};

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__CHAINER__LINE_CHAINER_HPP_
