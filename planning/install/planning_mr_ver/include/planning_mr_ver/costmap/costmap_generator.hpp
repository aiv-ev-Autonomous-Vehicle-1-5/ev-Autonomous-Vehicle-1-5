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
  CostmapResult generate(
    const std::vector<Point2D> & cones,
    const std::vector<Point2D> & lanes,
    const PlanningParams & params);

private:
  /// 단일 source의 자력을 grid에 적용 (MAX override)
  /// inner_radius: 이 반지름 내에서는 cost = cost_max (flat zone)
  ///               이 밖에서부터 1/r² 감쇠 시작
  static void apply_source(
    std::vector<double> & grid,
    int rows, int cols,
    double resolution,
    double origin_x, double origin_y,
    const Point2D & source,
    double cost_max,
    double alpha,
    double threshold,
    double inner_radius);

  /// decay zone의 유효 영향 반경 계산
  /// r_eff = sqrt((cost_max/threshold - 1) / alpha)
  static double effective_radius(
    double cost_max, double alpha, double threshold);
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__COSTMAP__COSTMAP_GENERATOR_HPP_
