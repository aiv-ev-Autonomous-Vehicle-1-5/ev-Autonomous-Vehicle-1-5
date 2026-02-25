/**
 * @file safety_checker.hpp
 * @brief 경로의 안전성/실현 가능성 검사 및 목표 속도 계산 헤더
 *
 * ## 역할
 * - 후처리된 경로(PostprocessResult)가 차량의 물리적 제약을 만족하는지 검사한다.
 * - 경로의 최대 곡률을 계산하고, 최소 회전 반경(R_min)과 비교한다.
 * - 곡률 기반 안전 속도와 v_max 중 작은 값을 목표 속도로 설정한다.
 *
 * ## SafetyResult 상태 우선순위 (높은 것부터)
 *  1. INFEASIBLE — 경로 곡률 > 차량 조향 한계 (경로 추종 불가)
 *  2. STOP      — 유효한 경로 없음
 *  3. OK        — 정상 주행 가능 (target_speed > 0)
 *
 * @note Stale 검사는 on_timer() 상단에서 사전 처리됨 (이 함수에 도달하면 항상 not-stale)
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │               Menger 곡률 (compute_max_curvature)               │
 * ├─────────────────────────────────────────────────────────────────┤
 * │                                                                 │
 * │           B(i+1)                                                │
 * │          / \                                                    │
 * │    ab   /   \  bc                                               │
 * │        /  O  \        O = 외접원 중심 (circumcenter)            │
 * │       / (R)   \       R = 외접원 반경 (circumradius)            │
 * │      /         \      κ = 1/R = 곡률                           │
 * │   A(i)────────C(i+2)                                           │
 * │        ac                                                       │
 * │                                                                 │
 * │  공식 유도:                                                     │
 * │    삼각형 넓이   Area = 0.5 * |cross(BA, CB)|                  │
 * │    외접원 반경   R = (ab * bc * ac) / (4 * Area)               │
 * │    곡률          κ = 1/R = 4*Area / (ab*bc*ac)                 │
 * │                     = 2*|cross(BA,CB)| / (ab*bc*ac)            │
 * │                                                                 │
 * │  물리적 제약:                                                   │
 * │    κ_limit = 1/r_min  (r_min = L/tan(δ_max), Ackermann)       │
 * │    κ_max > κ_limit → INFEASIBLE (조향 한계 초과)               │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * ## 목표 속도 계산
 *  원심 가속도 조건: a_lat = v² * κ ≤ a_lat_max
 *    → v ≤ sqrt(a_lat_max / κ)
 *
 *  v_curve = sqrt(a_lat_max / κ_max)
 *  v_target = min(v_max, v_curve)
 *  v_target = clamp(v_target, 0, v_max)
 *
 *  예시 (T870 기준: a_lat_max=2.0, v_max=1.6):
 *    κ=0.5 → v_curve=2.0 → v_target=1.6 (v_max 제한)
 *    κ=1.0 → v_curve=1.41 → v_target=1.41 (곡률 제한)
 *    κ=2.0 → v_curve=1.0 → v_target=1.0 (곡률 제한)
 */

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

/**
 * @struct SafetyResult
 * @brief check()의 출력 구조체
 *
 * @param state         플래너 상태 (OK / STOP / INFEASIBLE / STALE)
 * @param target_speed  목표 속도(m/s) — 0이면 정지
 * @param max_curvature 경로 최대 곡률(1/m)
 * @param reason        상태 설명 문자열 (디버깅/로깅용)
 */
struct SafetyResult
{
  PlannerState state = PlannerState::STOP;
  double target_speed = 0.0;
  double max_curvature = 0.0;
  std::string reason;
};

/**
 * @brief Menger 곡률로 경로의 최대 곡률 계산
 *
 * ## Menger 곡률 공식
 *  세 연속 포인트 A(i), B(i+1), C(i+2)에 대해:
 *
 *  BA = B - A  (벡터 BA)
 *  CB = C - B  (벡터 CB)
 *
 *  |cross(BA, CB)| = |BA.x * CB.y - BA.y * CB.x|
 *                  = 평행사변형 넓이 = 2 * 삼각형 ABС 넓이
 *
 *  κ = 2 * |cross(BA, CB)| / (|BA| * |CB| * |AC|)
 *
 *  분모 = 세 변의 길이 곱 (denom = ab * bc * ac)
 *  denom < 1e-12이면 세 점이 일직선 → κ = 0으로 건너뜀
 *
 *  모든 세점 조합 중 최대 κ를 반환
 *
 * @param path 경로 포인트 목록 (월드 좌표, 최소 3개 필요)
 * @return     경로 전체의 최대 곡률(1/m), 3개 미만이면 0.0
 */
inline double compute_max_curvature(const std::vector<Point2D> & path)
{
  // 3개 미만이면 곡률 계산 불가 (직선으로 간주)
  if (path.size() < 3) return 0.0;

  double kappa_max = 0.0;

  // 연속된 세 점(A=i, B=i+1, C=i+2)에 대해 곡률 계산
  for (size_t i = 0; i + 2 < path.size(); ++i) {
    const Point2D & a = path[i];       // 앞점
    const Point2D & b = path[i + 1];   // 중간점
    const Point2D & c = path[i + 2];   // 뒷점

    // 세 변의 길이 계산
    const double ab = dist(a, b);  // A→B 거리
    const double bc = dist(b, c);  // B→C 거리
    const double ac = dist(a, c);  // A→C 거리 (외접원 공식에 필요)

    const double denom = ab * bc * ac;  // 분모 = 세 변의 길이 곱
    // 분모가 너무 작으면 세 점이 거의 같거나 일직선 → 건너뜀
    if (denom < 1e-12) continue;

    // BA, CB 벡터 계산
    const Point2D ba = b - a;  // 벡터 B - A
    const Point2D cb = c - b;  // 벡터 C - B

    // 2D 외적(Z성분) = 삼각형 ABC 넓이의 2배
    // cross2(ba, cb) = ba.x * cb.y - ba.y * cb.x
    const double cross_val = std::abs(cross2(ba, cb));

    // Menger 곡률: κ = 2 * |cross| / (ab * bc * ac)
    const double kappa = 2.0 * cross_val / denom;

    // 최대 곡률 갱신
    if (kappa > kappa_max) {
      kappa_max = kappa;
    }
  }

  return kappa_max;
}

/**
 * @brief 경로의 안전성/실현 가능성 검사 및 목표 속도 계산
 *
 * ## 검사 순서 (우선순위 높은 것부터)
 *
 * ### 1. 경로 유효성 검사
 *   !path.valid || path.path.size() < 2이면:
 *     state = STOP, target_speed = 0, 즉시 반환
 *
 * ### 3. 곡률 실현 가능성 검사
 *   kappa_limit = 1 / r_min  (r_min = 최소 회전 반경)
 *   max_curvature > kappa_limit이면:
 *     state = INFEASIBLE, target_speed = 0
 *     reason = "curvature_exceeds_r_min"
 *     경로가 차량의 조향 한계를 초과 → 추종 불가
 *
 * ### 4. 목표 속도 계산
 *   v_target = v_max  (기본값)
 *
 *   [곡률 기반 속도 제한]
 *   κ_max > 1e-6이면:
 *     v_curve = sqrt(a_lat_max / κ_max)
 *              원심 가속도 = v² * κ ≤ a_lat_max
 *              → v ≤ sqrt(a_lat_max / κ)
 *     v_target = min(v_target, v_curve)
 *
 *   [클램프]
 *   v_target = clamp(v_target, 0, v_max)
 *
 *   state = OK, reason = "ok"
 *
 * @param path        후처리된 경로 결과 (PostprocessResult)
 * @param p           플래닝 파라미터 (v_max, a_lat_max, r_min)
 * @return            SafetyResult (상태, 목표속도, 최대곡률, 이유)
 */
inline SafetyResult check(
  const PostprocessResult & path,
  const PlanningParams & p)
{
  SafetyResult result;

  // ---- 1. 경로 유효성 검사 ----
  // 유효한 경로가 없거나 포인트 수가 너무 적으면 정지
  if (!path.valid || path.path.size() < 2) {
    result.state = PlannerState::STOP;
    result.reason = "no_valid_path";
    result.target_speed = 0.0;
    return result;
  }

  // ---- 2. 곡률 실현 가능성 검사 ----
  // Menger 곡률로 경로 전체의 최대 곡률 계산
  result.max_curvature = compute_max_curvature(path.path);

  // 최소 회전 반경에서 곡률 한계 계산
  // r_min = p.vehicle.r_min()   (차량 최소 회전 반경)
  // kappa_limit = 1 / r_min     (최대 허용 곡률)
  const double r_min = p.vehicle.r_min();
  const double kappa_limit = (r_min > 1e-6) ? (1.0 / r_min) : 1e6;  // r_min=0 방어

  if (result.max_curvature > kappa_limit) {
    // 경로 곡률이 차량 조향 한계를 초과 → 경로 추종 불가
    result.state = PlannerState::INFEASIBLE;
    result.target_speed = 0.0;
    result.reason = "curvature_exceeds_r_min";
    return result;
  }

  // ---- 3. 목표 속도 계산 ----
  double v_target = p.speed.v_max;  // 기본값: 최대 속도에서 시작

  // [곡률 기반 속도 제한]
  // 원심 가속도 조건: a_lat = v² * κ ≤ a_lat_max
  // → v ≤ sqrt(a_lat_max / κ)
  if (result.max_curvature > 1e-6) {
    // 곡률이 유의미한 경우만 속도 제한 계산 (0에 가까운 곡률 무시)
    const double v_curve = std::sqrt(p.speed.a_lat_max / result.max_curvature);
    v_target = std::min(v_target, v_curve);  // 더 작은 값으로 제한
  }

  // [최종 클램프]: v_target ∈ [0, v_max]
  v_target = std::max(0.0, std::min(v_target, p.speed.v_max));

  // 모든 검사 통과 → OK 상태
  result.state = PlannerState::OK;
  result.target_speed = v_target;
  result.reason = "ok";
  return result;
}

}  // namespace safety_checker
}  // namespace track_planning

#endif  // TRACK_PLANNING__SAFETY__SAFETY_CHECKER_HPP_
