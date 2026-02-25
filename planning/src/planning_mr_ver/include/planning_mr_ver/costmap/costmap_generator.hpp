/**
 * @file costmap_generator.hpp
 * @brief Magnetic Resistance Costmap 생성기 — 핵심 알고리즘 1
 *
 * ──────────────────────────────────────────────────────────────
 * 역할: 콘과 차선을 S극 자석으로 모델링하여 2D costmap 그리드를 생성
 *
 * 알고리즘 개요:
 *   1. ego(0,0) 중심으로 size_x × size_y 크기의 그리드 생성
 *   2. 각 콘/차선 점에 대해 주변 셀에 자력(cost)을 적용
 *   3. 콘: flat zone(반지름 내 최대 cost) + Gaussian decay zone
 *   4. 차선: flat zone 없음 (중심부터 바로 Gaussian 감쇠)
 *   5. 여러 자석이 겹치면 MAX override (가장 강한 값 유지)
 *
 * Gaussian 감쇠 공식 (nav2 costmap_2d 방식):
 *   d_eff = max(0, 거리 - inner_radius)
 *   cost = cost_max · exp(-d_eff² / (2σ²))
 *
 *   inner_radius 내부: cost = cost_max (flat zone)
 *   inner_radius 외부: Gaussian decay (3σ 밖은 ≈ 0)
 * ──────────────────────────────────────────────────────────────
 */
#ifndef PLANNING_MR_VER__COSTMAP__COSTMAP_GENERATOR_HPP_
#define PLANNING_MR_VER__COSTMAP__COSTMAP_GENERATOR_HPP_

#include "planning_mr_ver/common/types.hpp"
#include "planning_mr_ver/common/params.hpp"
#include <vector>

namespace planning_mr_ver
{

class CostmapGenerator
{
public:
  /**
   * @brief 콘과 차선 데이터로부터 costmap을 생성
   *
   * @param cones  콘 위치 목록 (ego 좌표계, LiDAR DBSCAN 결과)
   * @param lanes  차선 경계점 목록 (ego 좌표계, 카메라 인식 결과)
   * @param params 파라미터 (costmap 크기, 감쇠율, 최대 cost 등)
   * @return CostmapResult 생성된 costmap 그리드
   */
  CostmapResult generate(
    const std::vector<Point2D> & cones,
    const std::vector<Point2D> & lanes,
    const PlanningParams & params);

private:
  /**
   * @brief 단일 source(콘 또는 차선 점)의 자력을 grid에 적용 (MAX override)
   *
   * source를 중심으로 bounding box 범위의 셀을 순회하며 cost를 기록한다.
   *   - inner_radius 이내: cost = cost_max (flat zone, 물리적 장애물 크기)
   *   - inner_radius 밖: cost = cost_max · exp(-d_eff²/(2σ²)) (Gaussian decay)
   *   - threshold 미만: 0 처리 (건너뜀, 연산 절약)
   * 기존 값보다 큰 경우에만 덮어쓴다 (MAX override).
   *
   * @param sigma        Gaussian σ [m] (영향 범위 ≈ 3σ)
   * @param inner_radius flat zone 반지름 [m] (콘: 0.65m, 차선: 0.0m)
   */
  static void apply_source(
    std::vector<double> & grid,
    int rows, int cols,
    double resolution,
    double origin_x, double origin_y,
    const Point2D & source,
    double cost_max,
    double sigma,
    double threshold,
    double inner_radius);

  /**
   * @brief Gaussian decay zone의 유효 영향 반경 계산
   *
   * cost_max · exp(-r²/(2σ²)) = threshold 를 r에 대해 풀면:
   *   r = σ · sqrt(2 · ln(cost_max / threshold))
   *
   * @return decay zone만의 유효 반경 [m] (총 영향 반경 = inner_radius + 이 값)
   */
  static double effective_radius(
    double cost_max, double sigma, double threshold);
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__COSTMAP__COSTMAP_GENERATOR_HPP_
