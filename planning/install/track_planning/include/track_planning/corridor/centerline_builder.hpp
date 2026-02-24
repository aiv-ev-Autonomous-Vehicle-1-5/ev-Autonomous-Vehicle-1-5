#ifndef TRACK_PLANNING__CORRIDOR__CENTERLINE_BUILDER_HPP_
#define TRACK_PLANNING__CORRIDOR__CENTERLINE_BUILDER_HPP_

#include "track_planning/common/types.hpp"

#include <vector>

namespace track_planning
{

class CenterlineBuilder
{
public:
  /// Build centerline from corridor boundaries
  /// @param left          left boundary polyline
  /// @param right         right boundary polyline
  /// @param pair_valid    true if both boundaries are a valid pair
  /// @param virtual_used  true if one side is a virtual boundary
  /// @param visible_is_left  (used when virtual_used) which side is the real measurement
  /// @param w_hat         estimated track width (for virtual offset mode)
  /// @param resample_ds   resample interval for output centerline
  CenterlineResult build(
    const std::vector<Point2D> & left,
    const std::vector<Point2D> & right,
    bool pair_valid,
    bool virtual_used,
    bool visible_is_left,
    double w_hat,
    double resample_ds);

private:
  /// Case 1: both sides valid → midpoint averaging
  CenterlineResult build_from_pair(
    const std::vector<Point2D> & left,
    const std::vector<Point2D> & right,
    double resample_ds);

  /// Case 2: one-side + virtual → offset from visible side
  CenterlineResult build_from_one_side(
    const std::vector<Point2D> & visible,
    bool visible_is_left,
    double w_hat,
    double resample_ds);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__CORRIDOR__CENTERLINE_BUILDER_HPP_
