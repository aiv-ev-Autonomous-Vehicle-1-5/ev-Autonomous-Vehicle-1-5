#ifndef PLANNING_MR_VER__COMMON__TYPES_HPP_
#define PLANNING_MR_VER__COMMON__TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace planning_mr_ver
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct CostmapResult
{
  std::vector<double> data;   // row-major flat grid: data[row * cols + col]
  int rows = 0;
  int cols = 0;
  double resolution = 0.05;
  double origin_x = -5.0;    // cell(0,0)의 world x 좌표
  double origin_y = -5.0;    // cell(0,0)의 world y 좌표
  bool valid = false;
};

struct PostprocessResult
{
  std::vector<Point2D> path;
  std::vector<double> yaw;
  bool valid = false;
};

enum class PlannerState : uint8_t
{
  OK = 0,
  STOP = 1,
  INFEASIBLE = 2,
  STALE = 3
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__COMMON__TYPES_HPP_
