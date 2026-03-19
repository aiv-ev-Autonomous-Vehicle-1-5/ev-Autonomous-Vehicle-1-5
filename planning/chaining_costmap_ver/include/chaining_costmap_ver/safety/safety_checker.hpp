/**
 * @file safety_checker.hpp
 * @brief 안전 검사 모듈 — 경로 유효성 + 곡률(curvature) 검사
 *
 * ──────────────────────────────────────────────────────────────
 * [목적]
 *   경로(path)의 기하학적 실현 가능성을 판별한다.
 *
 *   1) 경로 유효성 검사: valid 플래그, 점 개수, 총 길이
 *   2) Menger 곡률 공식으로 경로 상 최대 곡률(κ_max)을 계산한다.
 *   3) κ_max가 차량의 최소 회전 반경(r_min)으로 결정되는 한계 곡률을
 *      초과하면 → WARNING (물리적으로 추종이 어려울 수 있는 경로).
 *
 * [반환값]
 *   SafetyResult { state, max_curvature, reason }
 *     - state: OK / FAIL / WARNING (PlannerState enum)
 *     - max_curvature: 경로 상 최대 곡률 [1/m]
 *     - reason: 상태 문자열 ("OK", "FAIL - <원인>", "WARNING - <원인>")
 * ──────────────────────────────────────────────────────────────
 */
#ifndef CHAINING_COSTMAP_VER__SAFETY__SAFETY_CHECKER_HPP_
#define CHAINING_COSTMAP_VER__SAFETY__SAFETY_CHECKER_HPP_

#include "chaining_costmap_ver/common/types.hpp"
#include "chaining_costmap_ver/common/params.hpp"
#include "chaining_costmap_ver/common/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace chaining_costmap_ver
{
namespace safety_checker
{

/**
 * @struct SafetyResult
 * @brief 안전 검사 결과를 담는 구조체
 *
 * - state:          플래너 상태 (OK=정상, FAIL=실패, WARNING=경고)
 * - max_curvature:  경로 전체에서 측정된 최대 곡률 [1/m]
 * - reason:         상태 문자열 ("OK", "FAIL - <원인>", "WARNING - <원인>")
 */
struct SafetyResult
{
  PlannerState state = PlannerState::FAIL;   ///< 기본값은 FAIL (안전 최우선)
  double max_curvature = 0.0;                ///< 경로 상 최대 곡률 [1/m]
  std::string reason;                        ///< 상태 문자열 ("OK", "FAIL - ...", "WARNING - ...")
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
 * @brief 경로의 총 길이를 계산 (유클리드 거리 합산)
 *
 * @param path  2D 점 벡터
 * @return      경로 총 길이 [m] (점이 2개 미만이면 0.0)
 */
inline double compute_path_length(const std::vector<Point2D> & path)
{
  if (path.size() < 2) return 0.0;
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += dist(path[i - 1], path[i]);
  }
  return length;
}

/**
 * @brief 후처리된 경로에 대해 안전 검사를 수행하고, 결과를 반환
 *
 * ──────────────────────────────────────────────────────────────
 * [검사 흐름]
 *
 *   1) 유효성 검사:  경로가 비었거나 점이 2개 미만이면
 *      → FAIL - no valid path
 *
 *   2) 최대 곡률 계산:  compute_max_curvature() 호출
 *
 *   3) 최소 회전 반경 제한:
 *      - κ_max > κ_limit 이면 → WARNING - curvature exceeds r_min
 *        (경로는 발행하되 주의 필요)
 *
 *   4) 최소 길이 검사: 경로 총 길이가 min_path_length 미만이면
 *      → WARNING - too short valid path
 *        (경로는 발행하되 짧은 경로임을 알림)
 *
 *   5) 모든 검사 통과 → OK
 *
 * @param path  후처리 결과 (PostprocessResult)
 * @param p     플래너 파라미터 (차량 사양, 안전 파라미터 등)
 * @return      SafetyResult (상태 + 최대 곡률 + 사유)
 * ──────────────────────────────────────────────────────────────
 */
inline SafetyResult check(
  const PostprocessResult & path,
  const PlanningParams & p)
{
  SafetyResult result;

  // ── 1) 유효성 검사: 경로가 없거나 너무 짧으면 실패 ──
  if (!path.valid || path.path.size() < 2) {
    result.state = PlannerState::FAIL;
    result.reason = "FAIL - no valid path";
    return result;
  }

  // ── 2) 최대 곡률 계산 (Menger 공식) ──
  //   (길이 검사보다 먼저 수행 — WARNING 판정에 곡률 정보 필요)
  result.max_curvature = compute_max_curvature(path.path);

  // ── 3) 최소 회전 반경 제한 검사 ──
  // r_min: 차량이 스티어링을 최대로 꺾었을 때의 최소 회전 반경 [m]
  // κ_limit = 1/r_min: 차량이 물리적으로 추종 가능한 최대 곡률 [1/m]
  const double r_min = p.vehicle.r_min();
  const double kappa_limit = (r_min > 1e-6) ? (1.0 / r_min) : 1e6;

  // 경로의 최대 곡률이 한계를 초과하면 → 경고 (추종은 시도하되 주의 필요)
  if (result.max_curvature > kappa_limit) {
    result.state = PlannerState::WARNING;
    result.reason = "WARNING - curvature exceeds r_min";
    return result;
  }

  // ── 4) 최소 길이 검사: 경로가 너무 짧으면 경고 ──
  // 경로는 발행하되, 제어기에 짧은 경로임을 알림
  const double path_length = compute_path_length(path.path);
  if (path_length < p.safety.min_path_length) {
    result.state = PlannerState::WARNING;
    result.reason = "WARNING - too short valid path";
    return result;
  }

  // ── 5) 모든 검사 통과 → OK ──
  result.state = PlannerState::OK;
  result.reason = "OK";
  return result;
}

}  // namespace safety_checker  — 안전 검사 모듈 끝
}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__SAFETY__SAFETY_CHECKER_HPP_
