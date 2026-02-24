#ifndef TRACK_PLANNING__COSTMAP__DRIVABLE_MASK_SCANLINE_HPP_
#define TRACK_PLANNING__COSTMAP__DRIVABLE_MASK_SCANLINE_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <cstdint>
#include <vector>

namespace track_planning
{

/// Build a drivable-area mask for A* planning using scanline fill
/// between left and right corridor polylines.
class DrivableMaskScanline
{
public:
  struct Result
  {
    std::vector<int8_t> grid;   // row-major: 0 = free, 100 = occupied
    int width = 0;
    int height = 0;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    bool valid = false;
  };

  /// Build drivable mask:
  ///  1. Init grid as occupied (100)
  ///  2. Scanline fill between corridor left/right → free (0)
  ///  3. Rasterize obstacles + cones (100)
  ///  4. Inflate obstacles
  Result build(
    const std::vector<Point2D> & corridor_left,
    const std::vector<Point2D> & corridor_right,
    const std::vector<Point2D> & cones,
    const std::vector<Point2D> & obstacles,
    const PlanningParams & p);

private:
  /// Interpolate Y value on a polyline at a given X (assumes polyline is roughly monotonic in X)
  /// Returns false if x is outside the polyline's X range.
  static bool interpolate_y_at_x(
    const std::vector<Point2D> & pts,
    double x, double & y_out);

  /// World coordinate → grid cell
  static bool world_to_grid(
    double wx, double wy,
    double origin_x, double origin_y, double resolution,
    int width, int height,
    int & col, int & row);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__COSTMAP__DRIVABLE_MASK_SCANLINE_HPP_
