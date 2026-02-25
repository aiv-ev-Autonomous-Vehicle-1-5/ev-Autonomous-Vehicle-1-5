#ifndef PLANNING_MR_VER__POSTPROCESS__PATH_POSTPROCESSOR_HPP_
#define PLANNING_MR_VER__POSTPROCESS__PATH_POSTPROCESSOR_HPP_

#include "planning_mr_ver/common/types.hpp"

#include <vector>

namespace planning_mr_ver
{

class PathPostprocessor
{
public:
  PostprocessResult process(
    const std::vector<Point2D> & raw_path,
    double prune_max_dev,
    int smooth_window,
    double resample_ds);

private:
  static std::vector<Point2D> prune(
    const std::vector<Point2D> & pts, double max_dev);

  static std::vector<Point2D> smooth(
    const std::vector<Point2D> & pts, int window);
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__POSTPROCESS__PATH_POSTPROCESSOR_HPP_
