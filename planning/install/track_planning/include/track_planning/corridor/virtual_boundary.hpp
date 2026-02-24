#ifndef TRACK_PLANNING__CORRIDOR__VIRTUAL_BOUNDARY_HPP_
#define TRACK_PLANNING__CORRIDOR__VIRTUAL_BOUNDARY_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <vector>

namespace track_planning
{

class VirtualBoundary
{
public:
  /// Generate the virtual (unseen) boundary from the visible boundary
  /// @param visible       the polyline of the visible side
  /// @param visible_is_left  true if the visible boundary is LEFT side
  /// @param w_hat         estimated track width
  /// @param p             planning parameters
  VirtualBoundaryResult generate(
    const std::vector<Point2D> & visible,
    bool visible_is_left,
    double w_hat,
    const PlanningParams & p);

  /// Update w_hat with EMA
  static double update_w_hat(double w_hat_prev, double width_measured, double alpha);

  /// Initialize w_hat: use measured width if previous pair was valid, else default
  static double init_w_hat(
    double prev_width_median,
    bool prev_pair_valid,
    double default_width);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__CORRIDOR__VIRTUAL_BOUNDARY_HPP_
