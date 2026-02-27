/**
 * @file safety_checker.hpp
 * @brief 안전 검사 모듈 — 곡률(curvature) 검사 및 속도 제한
 *
 * ──────────────────────────────────────────────────────────────
 * [목적]
 *   경로(path)의 기하학적 실현 가능성과 안전 속도를 판별한다.
 *
 *   1) Menger 곡률 공식으로 경로 상 최대 곡률(κ_max)을 계산한다.
 *   2) κ_max가 차량의 최소 회전 반경(r_min)으로 결정되는 한계 곡률을
 *      초과하면 → INFEASIBLE (물리적으로 추종 불가능한 경로).
 *   3) 곡률이 한계 내이면, 횡가속도(lateral acceleration) 제한으로부터
 *      안전 속도를 역산한다:  v = sqrt(a_lat_max / κ)
 *
 * [반환값]
 *   SafetyResult { state, target_speed, max_curvature, reason }
 *     - state: OK / STOP / INFEASIBLE (PlannerState enum)
 *     - target_speed: 추종 가능한 최대 속도 [m/s]
 *     - max_curvature: 경로 상 최대 곡률 [1/m]
 *     - reason: 사람이 읽을 수 있는 판정 사유 문자열
 * ──────────────────────────────────────────────────────────────
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

/**
 * @struct SafetyResult
 * @brief 안전 검사 결과를 담는 구조체
 *
 * - state:          플래너 상태 (OK=정상, STOP=정지 필요, INFEASIBLE=물리적 불가)
 * - target_speed:   이 경로에서 허용 가능한 최대 속도 [m/s]
 * - max_curvature:  경로 전체에서 측정된 최대 곡률 [1/m]
 * - reason:         판정 사유를 설명하는 문자열 (로그/디버그용)
 */
struct SafetyResult
{
  PlannerState state = PlannerState::STOP;   ///< 기본값은 STOP (안전 최우선)
  double target_speed = 0.0;                 ///< 목표 속도 [m/s]
  double max_curvature = 0.0;                ///< 경로 상 최대 곡률 [1/m]
  std::string reason;                        ///< 판정 사유 문자열
};

/**
 * @brief 경로 상 최대 곡률(κ_max)을 Menger 곡률 공식으로 계산
 *
 * ──────────────────────────────────────────────────────────────
 * [Menger 곡률 공식 유도]
 *
 *   세 점 A, B, C가 정의하는 외접원(circumscribed circle)의 반지름을 R이라 하면,
 *   삼각형 ABC에 대해 사인 법칙(law of sines)에 의해:
 *
 *       |AC| / sin(∠ABC) = 2R
 *
 *   한편, 삼각형의 넓이(area)는:
 *
 *       area = (1/2) |AB × BC|   (외적의 크기 = 2 * 삼각형 넓이)
 *
 *   사인 법칙에서 sin(∠ABC) = 2·area / (|AB|·|BC|) 을 대입하면:
 *
 *       2R = |AC| · |AB| · |BC| / (2 · area)
 *       R  = |AB| · |BC| · |AC| / (4 · area)
 *
 *   곡률 κ = 1/R 이므로:
 *
 *       κ = 4 · area / (|AB| · |BC| · |AC|)
 *         = 4 · (1/2)|AB × BC| / (|AB|·|BC|·|AC|)
 *         = 2 |AB × BC| / (|AB|·|BC|·|AC|)
 *
 *   이것이 아래 코드에서 사용하는 공식이다:
 *       kappa = 2 * |cross(BA, CB)| / (|AB| * |BC| * |AC|)
 *
 * [슬라이딩 윈도우 방식]
 *   연속된 세 점(i, i+1, i+2)에 대해 곡률을 구하고,
 *   경로 전체에서 최대값(κ_max)을 반환한다.
 *   → κ_max가 클수록 경로가 더 급격하게 꺾인다는 뜻.
 *
 * @param path  2D 점 벡터 (경로의 웨이포인트)
 * @return      경로 상 최대 곡률 [1/m] (점이 3개 미만이면 0.0)
 * ──────────────────────────────────────────────────────────────
 */
inline double compute_max_curvature(const std::vector<Point2D> & path)
{
  // 세 점이 있어야 곡률을 정의할 수 있다
  if (path.size() < 3) return 0.0;

  double kappa_max = 0.0;

  // 슬라이딩 윈도우: (A, B, C) = (path[i], path[i+1], path[i+2])
  for (size_t i = 0; i + 2 < path.size(); ++i) {
    const Point2D & a = path[i];       // 첫 번째 점 A
    const Point2D & b = path[i + 1];   // 두 번째 점 B (꼭짓점)
    const Point2D & c = path[i + 2];   // 세 번째 점 C

    // 세 변의 길이 계산
    const double ab = dist(a, b);  // |AB|
    const double bc = dist(b, c);  // |BC|
    const double ac = dist(a, c);  // |AC|

    // 분모 = |AB| * |BC| * |AC|
    // → 세 점이 겹치거나 너무 가까우면(denom ≈ 0) 건너뛴다
    const double denom = ab * bc * ac;
    if (denom < 1e-12) continue;

    // 벡터 BA = B - A, CB = C - B
    const Point2D ba = b - a;
    const Point2D cb = c - b;

    // 2D 외적의 절대값 = |BA × CB| = 삼각형 ABC 넓이의 2배
    const double cross_val = std::abs(cross2(ba, cb));

    // Menger 곡률: κ = 2|cross| / (|AB|·|BC|·|AC|)
    const double kappa = 2.0 * cross_val / denom;

    // 최대 곡률 갱신
    if (kappa > kappa_max) {
      kappa_max = kappa;
    }
  }

  return kappa_max;
}

/**
 * @brief 후처리된 경로에 대해 안전 검사를 수행하고, 결과를 반환
 *
 * ──────────────────────────────────────────────────────────────
 * [검사 흐름]
 *
 *   1) 유효성 검사:  경로가 비었거나 점이 2개 미만이면 → STOP
 *
 *   2) 최대 곡률 계산:  compute_max_curvature() 호출
 *
 *   3) 최소 회전 반경 제한:
 *      - r_min = 차량 파라미터에서 가져온 최소 회전 반경 [m]
 *      - κ_limit = 1 / r_min  (한계 곡률)
 *      - κ_max > κ_limit 이면 → INFEASIBLE
 *        (스티어링을 최대로 꺾어도 이 곡률을 따라갈 수 없다)
 *
 *   4) 횡가속도 기반 속도 제한:
 *      - 원운동에서:  a_lat = v² · κ
 *      - 허용 횡가속도 제한:  a_lat ≤ a_lat_max
 *      - 이를 v에 대해 풀면:  v ≤ sqrt(a_lat_max / κ)
 *      - 따라서 안전 속도:  v_curve = sqrt(a_lat_max / κ_max)
 *      - v_target = min(v_max, v_curve)
 *
 *   5) 최종 클램핑:  v_target을 [0, v_max] 범위로 제한
 *
 * @param path  후처리 결과 (PostprocessResult)
 * @param p     플래너 파라미터 (차량 사양, 속도 제한 등)
 * @return      SafetyResult (상태 + 목표 속도 + 최대 곡률 + 사유)
 * ──────────────────────────────────────────────────────────────
 */
inline SafetyResult check(
  const PostprocessResult & path,
  const PlanningParams & p)
{
  SafetyResult result;

  // ── 1) 유효성 검사: 경로가 없거나 너무 짧으면 정지 ──
  if (!path.valid || path.path.size() < 2) {
    result.state = PlannerState::STOP;
    result.reason = "no_valid_path";
    result.target_speed = 0.0;
    return result;
  }

  // ── 2) 최대 곡률 계산 (Menger 공식) ──
  result.max_curvature = compute_max_curvature(path.path);

  // ── 3) 최소 회전 반경 제한 검사 ──
  // r_min: 차량이 스티어링을 최대로 꺾었을 때의 최소 회전 반경 [m]
  // κ_limit = 1/r_min: 차량이 물리적으로 추종 가능한 최대 곡률 [1/m]
  const double r_min = p.vehicle.r_min();
  const double kappa_limit = (r_min > 1e-6) ? (1.0 / r_min) : 1e6;

  // 경로의 최대 곡률이 한계를 초과하면 → 물리적으로 추종 불가능
  if (result.max_curvature > kappa_limit) {
    result.state = PlannerState::INFEASIBLE;
    result.target_speed = 0.0;
    result.reason = "curvature_exceeds_r_min";
    return result;
  }

  // ── 4) 횡가속도 기반 안전 속도 계산 ──
  // 초기값: 최대 허용 속도
  double v_target = p.speed.v_max;

  // 곡률이 유의미한 크기(>1e-6)이면 속도 제한을 적용
  // 원운동 공식: a_lat = v² * κ  →  v = sqrt(a_lat_max / κ)
  // 이 속도를 초과하면 차가 미끄러진다 (타이어 그립 한계 초과)
  if (result.max_curvature > 1e-6) {
    const double v_curve = std::sqrt(p.speed.a_lat_max / result.max_curvature);
    v_target = std::min(v_target, v_curve);
  }

  // ── 5) 최종 클램핑: [0, v_max] 범위로 제한 ──
  v_target = std::max(0.0, std::min(v_target, p.speed.v_max));

  result.state = PlannerState::OK;
  result.target_speed = v_target;
  result.reason = "ok";
  return result;
}

}  // namespace safety_checker  — 안전 검사 모듈 끝
}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__SAFETY__SAFETY_CHECKER_HPP_
