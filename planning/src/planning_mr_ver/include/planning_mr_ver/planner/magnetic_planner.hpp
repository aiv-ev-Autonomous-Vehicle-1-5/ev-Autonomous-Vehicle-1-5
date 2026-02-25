#ifndef PLANNING_MR_VER__PLANNER__MAGNETIC_PLANNER_HPP_
#define PLANNING_MR_VER__PLANNER__MAGNETIC_PLANNER_HPP_

#include "planning_mr_ver/common/types.hpp"
#include "planning_mr_ver/common/params.hpp"
#include <vector>

namespace planning_mr_ver
{

class MagneticPlanner
{
public:
  std::vector<Point2D> plan(
    const CostmapResult & costmap,
    const PlanningParams & params);

private:
  static bool world_to_grid(
    double wx, double wy,
    double origin_x, double origin_y,
    double resolution,
    int rows, int cols,
    int & row, int & col);

  static Point2D grid_to_world(
    int row, int col,
    double origin_x, double origin_y,
    double resolution);

  /// 전방 180° 내에서 가장 낮은 cost의 cell을 찾는다.
  /// cost 동률 시 heading 방향에 가까운 cell 우선 (oscillation 방지).
  static Point2D find_best_forward_cell(
    const CostmapResult & costmap,
    const Point2D & current_pos,
    const Point2D & heading,
    const PlanningParams & params,
    bool & found);
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__PLANNER__MAGNETIC_PLANNER_HPP_
