#ifndef TRACK_PLANNING__COSTMAP__CONNECTED_COMPONENT_HPP_
#define TRACK_PLANNING__COSTMAP__CONNECTED_COMPONENT_HPP_

#include "track_planning/common/types.hpp"

#include <cstdint>
#include <vector>

namespace track_planning
{

/// BFS-based connected component filter.
/// Keeps only free cells reachable from ego position; unreachable free cells → occupied.
class ConnectedComponent
{
public:
  /// Filter grid in-place: free cells not reachable from ego → occupied (100)
  /// @param grid       row-major grid (0 = free, 100 = occupied)
  /// @param ego_pos    ego position in world coordinates
  static void filter_ego_component(
    std::vector<int8_t> & grid,
    int width, int height,
    double resolution, double origin_x, double origin_y,
    const Point2D & ego_pos);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__COSTMAP__CONNECTED_COMPONENT_HPP_
