/**
 * @file safety_checker.hpp
 * @brief 안전 검사 모듈 — 곡률 검사 및 속도 제한
 */
#ifndef PLANNING_LC_VER__SAFETY__SAFETY_CHECKER_HPP_
#define PLANNING_LC_VER__SAFETY__SAFETY_CHECKER_HPP_

#include "planning_lc_ver/common/types.hpp"
#include "planning_lc_ver/common/params.hpp"
#include "planning_lc_ver/common/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace planning_lc_ver
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

inline SafetyResult check(
  const PostprocessResult & path,
  const PlanningParams & p)
{
  SafetyResult result;

  if (!path.valid || path.path.size() < 2) {
    result.state = PlannerState::STOP;
    result.reason = "no_valid_path";
    result.target_speed = 0.0;
    return result;
  }

  result.max_curvature = compute_max_curvature(path.path);

  const double r_min = p.vehicle.r_min();
  const double kappa_limit = (r_min > 1e-6) ? (1.0 / r_min) : 1e6;

  if (result.max_curvature > kappa_limit) {
    result.state = PlannerState::INFEASIBLE;
    result.target_speed = 0.0;
    result.reason = "curvature_exceeds_r_min";
    return result;
  }

  double v_target = p.speed.v_max;

  if (result.max_curvature > 1e-6) {
    const double v_curve = std::sqrt(p.speed.a_lat_max / result.max_curvature);
    v_target = std::min(v_target, v_curve);
  }

  v_target = std::max(0.0, std::min(v_target, p.speed.v_max));

  result.state = PlannerState::OK;
  result.target_speed = v_target;
  result.reason = "ok";
  return result;
}

}  // namespace safety_checker
}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__SAFETY__SAFETY_CHECKER_HPP_
