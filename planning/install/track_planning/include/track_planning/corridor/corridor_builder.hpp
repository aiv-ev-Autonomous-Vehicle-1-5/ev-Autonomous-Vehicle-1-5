#ifndef TRACK_PLANNING__CORRIDOR__CORRIDOR_BUILDER_HPP_
#define TRACK_PLANNING__CORRIDOR__CORRIDOR_BUILDER_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <utility>
#include <vector>

namespace track_planning
{

class CorridorBuilder
{
public:
  struct Input
  {
    std::vector<Point2D> lane_left;
    std::vector<Point2D> lane_right;
    std::vector<Point2D> cone_left;
    std::vector<Point2D> cone_right;
    Point2D ego_pos{0.0, 0.0};
    Point2D ego_heading{1.0, 0.0};        // unit vector, default = +x
    std::vector<Point2D> centerline_prev;  // empty = cold start
  };

  CorridorPolylines build(const Input & in, const PlanningParams & p);

private:
  /// Determine reference point (c_end) and tangent (t_end) for chaining
  std::pair<Point2D, Point2D> determine_reference(
    const Input & in, const PlanningParams & p);

  /// Build one side (left or right) polyline via greedy chaining
  std::vector<Point2D> build_one_side(
    const std::vector<Point2D> & lane_pts,
    const std::vector<Point2D> & cone_pts,
    const Point2D & c_end,
    const Point2D & t_end,
    const Point2D & ego_pos,
    const PlanningParams & p);

  /// Find seed point: closest to ego among pts with x >= x_min
  bool find_seed(
    const std::vector<Point2D> & lane_pts,
    const std::vector<Point2D> & cone_pts,
    const Point2D & ego,
    double x_min,
    Point2D & seed_out);

  /// Filter candidates from a point set, returning valid indices
  std::vector<size_t> filter_candidates(
    const std::vector<Point2D> & pts,
    const Point2D & c_end,
    const Point2D & t_end,
    const Point2D & p_k,
    const PlanningParams & p);

  /// Score candidates and return best index (-1 if none)
  int score_and_select(
    const std::vector<Point2D> & pts,
    const std::vector<size_t> & candidates,
    const Point2D & c_end,
    const Point2D & t_end,
    const Point2D & p_k,
    const Point2D & t_k,
    const PlanningParams & p);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__CORRIDOR__CORRIDOR_BUILDER_HPP_
