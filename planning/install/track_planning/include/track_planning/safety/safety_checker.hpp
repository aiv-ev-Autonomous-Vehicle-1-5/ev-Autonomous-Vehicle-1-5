#ifndef TRACK_PLANNING__SAFETY__SAFETY_CHECKER_HPP_
#define TRACK_PLANNING__SAFETY__SAFETY_CHECKER_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"
#include "track_planning/common/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace track_planning
{
namespace safety_checker
{

struct SafetyResult
{
  PlannerState state = PlannerState::STOP;
  double target_speed = 0.0;
  double max_curvature = 0.0;
  std::string reason;
};

/// Compute maximum curvature along a path using Menger curvature.
/// κ = 2|cross(B-A, C-B)| / (|B-A| · |C-B| · |A-C|)
inline double compute_max_curvature(const std::vector<Point2D> & path)
{
  if (path.size() < 3) return 0.0;

  double kappa_max = 0.0;

  for (size_t i = 0; i + 2 < path.size(); ++i) {
    const Point2D & a = path[i];
    const Point2D & b = path[i + 1];
    const Point2D & c = path[i + 2];

    const double ab = dist(a, b);
    const double bc = dist(b, c);
    const double ac = dist(a, c);

    const double denom = ab * bc * ac;
    if (denom < 1e-12) continue;

    // 2 * |cross(B-A, C-B)| / (|B-A| * |C-B| * |A-C|)
    const Point2D ba = b - a;
    const Point2D cb = c - b;
    const double cross_val = std::abs(cross2(ba, cb));
    const double kappa = 2.0 * cross_val / denom;

    if (kappa > kappa_max) {
      kappa_max = kappa;
    }
  }

  return kappa_max;
}

/// Run safety / feasibility check and compute target speed.
/// @param path          Post-processed path result
/// @param input_stale   True if any input data has timed out
/// @param astar_failed  True if A* was attempted but failed
/// @param p             Planning parameters
inline SafetyResult check(
  const PostprocessResult & path,
  bool input_stale,
  bool astar_failed,
  const PlanningParams & p)
{
  SafetyResult result;

  // ---- Stale gate (highest priority) ----
  if (input_stale) {
    result.state = PlannerState::STALE;
    result.target_speed = 0.0;
    result.reason = "input_stale";
    return result;
  }

  // ---- Path validity ----
  if (!path.valid || path.path.size() < 2) {
    if (astar_failed) {
      result.state = PlannerState::INFEASIBLE;
      result.reason = "astar_failed";
    } else {
      result.state = PlannerState::STOP;
      result.reason = "no_valid_path";
    }
    result.target_speed = 0.0;
    return result;
  }

  // ---- Curvature feasibility ----
  result.max_curvature = compute_max_curvature(path.path);

  const double r_min = p.vehicle.r_min();
  const double kappa_limit = (r_min > 1e-6) ? (1.0 / r_min) : 1e6;

  if (result.max_curvature > kappa_limit) {
    result.state = PlannerState::INFEASIBLE;
    result.target_speed = 0.0;
    result.reason = "curvature_exceeds_r_min";
    return result;
  }

  // ---- Target speed computation ----
  double v_target = p.speed.v_max;

  // Curvature-based speed limit: v = sqrt(a_lat_max / kappa)
  if (result.max_curvature > 1e-6) {
    const double v_curve = std::sqrt(p.speed.a_lat_max / result.max_curvature);
    v_target = std::min(v_target, v_curve);
  }

  // Clamp to [0, v_max]
  v_target = std::max(0.0, std::min(v_target, p.speed.v_max));

  result.state = PlannerState::OK;
  result.target_speed = v_target;
  result.reason = "ok";
  return result;
}

}  // namespace safety_checker
}  // namespace track_planning

#endif  // TRACK_PLANNING__SAFETY__SAFETY_CHECKER_HPP_
