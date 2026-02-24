#ifndef TRACK_PLANNING__COMMON__TYPES_HPP_
#define TRACK_PLANNING__COMMON__TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace track_planning
{

// ---- Primitive ----

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

// ---- Corridor ----

struct CorridorPolylines
{
  std::vector<Point2D> left;
  std::vector<Point2D> right;
  bool left_ok = false;
  bool right_ok = false;
};

struct PairResult
{
  bool valid = false;
  double width_median = 0.0;
  double width_std = 0.0;
  double angle_mean = 0.0;
};

struct VirtualBoundaryResult
{
  std::vector<Point2D> boundary;
  bool success = false;
};

struct CenterlineResult
{
  std::vector<Point2D> center;
  bool valid = false;
};

// ---- Costmap / Path ----

enum class PathMode : uint8_t
{
  DIRECT = 0,
  ASTAR = 1
};

struct GoalResult
{
  Point2D goal;
  bool valid = false;
  uint8_t method = 0;   // 0 = centerline-based, 1 = ring sampling
  double score = 0.0;
};

// ---- Postprocess ----

struct PostprocessResult
{
  std::vector<Point2D> path;
  std::vector<double> yaw;   // heading at each point (rad)
  bool valid = false;
};

// ---- Planner Status ----

enum class PlannerState : uint8_t
{
  OK = 0,
  STOP = 1,
  INFEASIBLE = 2,
  STALE = 3
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__COMMON__TYPES_HPP_
