/**
 * @file costmap_generator.hpp
 * @brief Magnetic Resistance Costmap 생성기 — LineChainer 체인 입력 버전
 *
 * 기존 mr_ver에서는 cones/lanes를 별도 벡터로 받았지만,
 * lc_ver에서는 LineChainer가 출력한 좌/우 ChainedPoint 체인을 직접 받는다.
 * ChainedPoint.type에 따라 콘/차선 각각의 cost 파라미터를 적용한다.
 */
#ifndef PLANNING_LC_VER__COSTMAP__COSTMAP_GENERATOR_HPP_
#define PLANNING_LC_VER__COSTMAP__COSTMAP_GENERATOR_HPP_

#include "planning_lc_ver/common/types.hpp"
#include "planning_lc_ver/common/params.hpp"
#include <vector>

namespace planning_lc_ver
{

class CostmapGenerator
{
public:
  /**
   * @brief 좌/우 체인으로부터 costmap을 생성
   *
   * @param left_chain  좌측 경계 체인 (리샘플 완료, ChainedPoint 타입 포함)
   * @param right_chain 우측 경계 체인
   * @param params      파라미터
   * @return CostmapResult 생성된 costmap 그리드
   */
  CostmapResult generate(
    const std::vector<ChainedPoint> & left_chain,
    const std::vector<ChainedPoint> & right_chain,
    const PlanningParams & params);

private:
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

  static double effective_radius(
    double cost_max, double sigma, double threshold);
};

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__COSTMAP__COSTMAP_GENERATOR_HPP_
