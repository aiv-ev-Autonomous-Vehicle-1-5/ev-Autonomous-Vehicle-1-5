#ifndef TRACK_PLANNING__POSTPROCESS__PATH_POSTPROCESSOR_HPP_
#define TRACK_PLANNING__POSTPROCESS__PATH_POSTPROCESSOR_HPP_

#include "track_planning/common/types.hpp"

#include <vector>

namespace track_planning
{

/// Post-process a raw path: prune → smooth → resample → compute yaw.
class PathPostprocessor
{
public:
  /// Run the full postprocess pipeline.
  /// @param raw_path       Input path (from A* or centerline)
  /// @param prune_max_dev  Maximum lateral deviation for shortcut pruning (m)
  /// @param smooth_window  Moving-average window size (odd recommended)
  /// @param resample_ds    Output uniform spacing (m)
  PostprocessResult process(
    const std::vector<Point2D> & raw_path,
    double prune_max_dev,
    int smooth_window,
    double resample_ds);

private:
  /// Greedy shortcut pruning: remove redundant waypoints
  static std::vector<Point2D> prune(
    const std::vector<Point2D> & pts, double max_dev);

  /// Moving-average smoothing (endpoints preserved)
  static std::vector<Point2D> smooth(
    const std::vector<Point2D> & pts, int window);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__POSTPROCESS__PATH_POSTPROCESSOR_HPP_
