#ifndef TRACK_PLANNING__GOAL__GOAL_SELECTOR_HPP_
#define TRACK_PLANNING__GOAL__GOAL_SELECTOR_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <cstdint>
#include <vector>

namespace track_planning
{

/// Select a goal cell for A* planning.
///  Method 1: centerline/path_prev lookahead
///  Method 2: free-space ring sampling (fallback)
class GoalSelector
{
public:
  GoalResult select(
    const std::vector<Point2D> & centerline,
    const std::vector<Point2D> & path_prev,
    const std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    const Point2D & ego_pos,
    const Point2D & ego_heading,
    double ego_speed,
    const PlanningParams & p);

private:
  /// Method 1: lookahead along a reference polyline
  static GoalResult method_lookahead(
    const std::vector<Point2D> & ref_line,
    const std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    double lookahead,
    uint8_t method_id);

  /// Method 2: ring sampling around ego
  static GoalResult method_ring(
    const std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    const Point2D & ego_pos,
    const Point2D & ego_heading,
    double lookahead,
    int n_samples,
    const std::vector<Point2D> & ref_line);

  /// Check if a world point is free in the grid
  static bool is_free(
    const std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    const Point2D & pt);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__GOAL__GOAL_SELECTOR_HPP_
