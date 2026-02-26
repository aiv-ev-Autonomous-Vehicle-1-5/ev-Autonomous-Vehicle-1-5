/**
 * @file magnetic_planner.hpp
 * @brief Magnetic Resistance Planner — costmap 위 Greedy 전진 탐색
 */
#ifndef PLANNING_LC_VER__PLANNER__MAGNETIC_PLANNER_HPP_
#define PLANNING_LC_VER__PLANNER__MAGNETIC_PLANNER_HPP_

#include "planning_lc_ver/common/types.hpp"
#include "planning_lc_ver/common/params.hpp"
#include <vector>

namespace planning_lc_ver
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

  static Point2D find_best_forward_cell(
    const CostmapResult & costmap,
    const Point2D & current_pos,
    const Point2D & heading,
    const PlanningParams & params,
    bool & found);
};

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__PLANNER__MAGNETIC_PLANNER_HPP_
