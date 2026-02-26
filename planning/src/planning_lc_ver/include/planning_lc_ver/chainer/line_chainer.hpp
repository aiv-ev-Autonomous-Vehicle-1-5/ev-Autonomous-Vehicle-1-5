/**
 * @file line_chainer.hpp
 * @brief LineChainer — Flood fill 기반 경계점 체이닝
 *
 * ──────────────────────────────────────────────────────────────
 * 역할: 콘/차선 점들을 좌/우 경계 점 집합으로 분류 + edge별 리샘플링
 *
 * 알고리즘 흐름:
 *   1. Seed 선택: y>0인 점 중 ego 최근접 → left seed
 *                  y<0인 점 중 ego 최근접 → right seed
 *   2. Flood fill: 각 seed에서 탐색 반경 내 모든 점을 감염
 *      - visited 배열을 좌/우 공유 → 중복 소속 방지
 *      - 후보가 없으면 search_radius 확장 (max까지)
 *      - Cone priority: 콘+차선 혼합 시 차선 스킵
 *      - parent→child edge 기록 → edge별 보간점 삽입
 *   3. 리샘플링: edge별 0.1m 보간, cone-cone 구간은 CONE 타입 유지
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
   * @brief 모든 경계점을 좌/우 체인으로 분류 (flood fill + edge별 리샘플링)
   *
   * @param candidates 모든 경계점 (콘+차선, 타입 태그 포함)
   * @param params     체이너 파라미터 (search_radius 등)
   * @return ChainResult 좌/우 체인 (edge별 보간점 포함) + valid 플래그
   */
  ChainResult chain(
    const std::vector<ChainedPoint> & candidates,
    const PlanningParams & params);

private:
  /**
   * @brief 한쪽(좌 또는 우) 점 집합을 flood fill로 수집 + edge별 리샘플링
   *
   * seed에서 출발하여 탐색 반경 내 모든 미방문 점을 감염시킨다.
   * visited 배열은 좌/우 chain이 공유하여 중복 소속을 방지한다.
   * parent→child edge를 기록하여 각 edge 사이에 보간점을 삽입한다.
   *
   * @param candidates 모든 후보점
   * @param seed_idx   시작점 인덱스
   * @param visited    방문 배열 (공유)
   * @param params     파라미터
   * @return 원본 점 + edge별 보간점
   */
  static std::vector<ChainedPoint> chain_one_side(
    const std::vector<ChainedPoint> & candidates,
    int seed_idx,
    std::vector<bool> & visited,
    const PlanningParams & params);
};

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__CHAINER__LINE_CHAINER_HPP_
