#ifndef TRACK_PLANNING__PLANNER__ASTAR_PLANNER_HPP_
#define TRACK_PLANNING__PLANNER__ASTAR_PLANNER_HPP_

#include "track_planning/common/types.hpp"

#include <cstdint>
#include <vector>

namespace track_planning
{

/// 8-neighbor A* grid planner on a drivable mask grid.
class AstarPlanner
{
public:
  struct Result
  {
    std::vector<Point2D> path;   // world coordinates
    bool success = false;
    int iterations = 0;
  };

  /// Plan a path from start to goal on the grid.
  /// @param grid              row-major (0 = free, >0 = cost/occupied)
  /// @param max_iterations    iteration budget (0 = unlimited)
  /// @param cell_cost_weight  weight for cell cost in g-score computation
  Result plan(
    const std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    const Point2D & start,
    const Point2D & goal,
    int max_iterations,
    double cell_cost_weight);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__PLANNER__ASTAR_PLANNER_HPP_
