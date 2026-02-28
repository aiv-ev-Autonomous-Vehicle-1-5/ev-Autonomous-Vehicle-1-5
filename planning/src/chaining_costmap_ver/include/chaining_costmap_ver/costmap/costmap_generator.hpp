/**
 * @file costmap_generator.hpp
 * @brief 가우시안 비용 지도(Costmap) 생성기
 *
 * 경계점(콘, 차선)을 "자석"처럼 취급하여, 각 점이 주변 공간에
 * 가우시안 비용을 방사하는 2D 격자 비용 지도를 생성한다.
 *
 * [가우시안 비용 공식]
 *   cost(d) = cost_max * exp( -d² / (2 * sigma²) )
 *
 * [CONE vs LANE]
 *   - CONE: cone_cost_max(100) + cone_radius flat zone → 강한 회피
 *   - LANE: lane_cost_max(50) + flat zone 없음 → 차선을 넘을 수 있음
 *
 * [unchained 포인트]
 *   체이닝에 실패한 점들은 보수적으로 CONE 비용으로 처리.
 */
#ifndef CHAINING_COSTMAP_VER__COSTMAP__COSTMAP_GENERATOR_HPP_
#define CHAINING_COSTMAP_VER__COSTMAP__COSTMAP_GENERATOR_HPP_

#include "chaining_costmap_ver/common/types.hpp"
#include "chaining_costmap_ver/common/params.hpp"
#include <vector>

namespace chaining_costmap_ver
{

class CostmapGenerator
{
public:
  /**
   * @brief 좌/우 체인으로부터 costmap 생성
   */
  CostmapResult generate(
    const std::vector<ChainedPoint> & left_chain,
    const std::vector<ChainedPoint> & right_chain,
    const PlanningParams & params);

  /**
   * @brief 좌/우 체인 + unchained 포인트로 costmap 생성
   *
   * unchained 포인트는 체이닝에 실패한 점들로,
   * CONE 비용(cone_cost_max)으로 보수적 처리한다.
   */
  CostmapResult generate(
    const std::vector<ChainedPoint> & left_chain,
    const std::vector<ChainedPoint> & right_chain,
    const std::vector<ChainedPoint> & unchained,
    const PlanningParams & params);

  /**
   * @brief 시드→ego 양옆까지 cone_cost_max 벽을 그려서 입구로 유도
   *
   * 좌/우 시드에서 ego(x=0)까지 벽을 연장하여
   * A*가 시드 사이(입구)로만 진입할 수 있게 한다.
   */
  static void apply_entry_walls(
    CostmapResult & costmap,
    const Point2D & left_seed,
    const Point2D & right_seed,
    const PlanningParams & params);

private:
  /**
   * @brief 단일 경계점의 가우시안 비용장을 그리드에 적용 (max-merge)
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
   * @brief 비용이 threshold 이상인 최대 거리 계산 (가우시안 역함수)
   */
  static double effective_radius(
    double cost_max, double sigma, double threshold);
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__COSTMAP__COSTMAP_GENERATOR_HPP_
