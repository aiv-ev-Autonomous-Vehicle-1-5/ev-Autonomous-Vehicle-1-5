/**
 * @file safety_checker.hpp
 * @brief 안전 검사 모듈 — 곡률 검사 및 속도 제한
 *
 * ──────────────────────────────────────────────────────────────
 * 역할: 후처리된 경로가 차량의 물리적 한계 내에서 주행 가능한지 검사
 *
 * 검사 항목:
 *   1. 유효성 검사: 경로가 존재하고 점이 2개 이상인지
 *   2. 곡률 검사: 최대 곡률이 차량 최소 회전반경(r_min) 이내인지
 *   3. 속도 제한: 곡률 기반 속도 계산 v = sqrt(a_lat_max / kappa)
 *
 * 결과:
 *   - OK: 정상 주행 가능, target_speed 포함
 *   - STOP: 유효한 경로 없음
 *   - INFEASIBLE: 곡률이 차량 한계 초과
 * ──────────────────────────────────────────────────────────────
 */
#ifndef PLANNING_MR_VER__SAFETY__SAFETY_CHECKER_HPP_
#define PLANNING_MR_VER__SAFETY__SAFETY_CHECKER_HPP_

#include "planning_mr_ver/common/types.hpp"
#include "planning_mr_ver/common/params.hpp"
#include "planning_mr_ver/common/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace planning_mr_ver
{
namespace safety_checker
{

/**
 * @brief 안전 검사 결과 구조체
 */
struct SafetyResult
{
  PlannerState state = PlannerState::STOP;  ///< 플래너 상태 (OK/STOP/INFEASIBLE)
  double target_speed = 0.0;                ///< 곡률 기반 목표 속도 [m/s]
  double max_curvature = 0.0;               ///< 경로 최대 곡률 [1/m]
  std::string reason;                       ///< 상태 사유 문자열
};

/**
 * @brief 경로의 최대 곡률을 계산
 *
 * 연속 3점(a, b, c)에서 곡률을 계산하고, 전체 경로에서 최대값을 반환한다.
 *
 * 곡률 공식 (외접원 기반):
 *   kappa = 2 * |cross(b-a, c-b)| / (|ab| * |bc| * |ac|)
 *
 * 이 공식은 삼각형 ABC의 외접원 반지름 R의 역수와 같다:
 *   kappa = 1/R, R = (|ab| * |bc| * |ac|) / (4 * 삼각형 넓이)
 *
 * @param path 곡률을 계산할 경로 점 목록
 * @return 최대 곡률 [1/m] (경로가 3점 미만이면 0)
 */
inline double compute_max_curvature(const std::vector<Point2D> & path)
{
  if (path.size() < 3) return 0.0;  // 3점 미만이면 곡률 계산 불가

  double kappa_max = 0.0;

  for (size_t i = 0; i + 2 < path.size(); ++i) {
    const Point2D & a = path[i];
    const Point2D & b = path[i + 1];
    const Point2D & c = path[i + 2];

    // 세 변의 길이
    const double ab = dist(a, b);
    const double bc = dist(b, c);
    const double ac = dist(a, c);

    // 분모: |ab| * |bc| * |ac| (거의 0이면 직선 또는 겹침 → 건너뜀)
    const double denom = ab * bc * ac;
    if (denom < 1e-12) continue;

    // 분자: 2 * |외적| = 4 * 삼각형 넓이
    const Point2D ba = b - a;
    const Point2D cb = c - b;
    const double cross_val = std::abs(cross2(ba, cb));

    // 곡률 = 2 * |외적| / (|ab| * |bc| * |ac|)
    const double kappa = 2.0 * cross_val / denom;

    if (kappa > kappa_max) {
      kappa_max = kappa;
    }
  }

  return kappa_max;
}

/**
 * @brief 안전 검사 수행 — 곡률 검사 및 속도 제한 계산
 *
 * 검사 순서:
 *   1. 경로 유효성: path가 없거나 점 < 2 → STOP
 *   2. 곡률 한계: kappa_max > 1/r_min → INFEASIBLE
 *   3. 속도 계산: v_curve = sqrt(a_lat_max / kappa)
 *      → v_max와 비교하여 더 작은 값 채택
 *   4. 정상: OK + target_speed 반환
 *
 * 속도 제한 원리 (원심력):
 *   a_lat = v² * kappa ≤ a_lat_max
 *   → v ≤ sqrt(a_lat_max / kappa)
 *
 * @param path 후처리된 경로 결과
 * @param p    파라미터 (차량 제원, 속도 제한)
 * @return SafetyResult 검사 결과
 */
inline SafetyResult check(
  const PostprocessResult & path,
  const PlanningParams & p)
{
  SafetyResult result;

  // ── 유효성 검사 ──
  if (!path.valid || path.path.size() < 2) {
    result.state = PlannerState::STOP;
    result.reason = "no_valid_path";
    result.target_speed = 0.0;
    return result;
  }

  // ── 최대 곡률 계산 ──
  result.max_curvature = compute_max_curvature(path.path);

  // ── 곡률 한계 검사 ──
  // r_min = wheelbase / tan(delta_max): 차량의 물리적 최소 회전반경
  // kappa_limit = 1 / r_min: 차량이 따라갈 수 있는 최대 곡률
  const double r_min = p.vehicle.r_min();
  const double kappa_limit = (r_min > 1e-6) ? (1.0 / r_min) : 1e6;

  if (result.max_curvature > kappa_limit) {
    // 경로의 곡률이 차량 최소 회전반경을 초과 → 물리적으로 불가능
    result.state = PlannerState::INFEASIBLE;
    result.target_speed = 0.0;
    result.reason = "curvature_exceeds_r_min";
    return result;
  }

  // ── 곡률 기반 속도 제한 계산 ──
  // 원심력 공식: a_lat = v² * kappa
  // 최대 속도: v = sqrt(a_lat_max / kappa)
  double v_target = p.speed.v_max;

  if (result.max_curvature > 1e-6) {
    // 곡률이 있으면 (직선이 아니면) 속도 제한 적용
    const double v_curve = std::sqrt(p.speed.a_lat_max / result.max_curvature);
    v_target = std::min(v_target, v_curve);  // v_max와 v_curve 중 작은 값
  }

  // 범위 제한: [0, v_max]
  v_target = std::max(0.0, std::min(v_target, p.speed.v_max));

  result.state = PlannerState::OK;
  result.target_speed = v_target;
  result.reason = "ok";
  return result;
}

}  // namespace safety_checker
}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__SAFETY__SAFETY_CHECKER_HPP_
