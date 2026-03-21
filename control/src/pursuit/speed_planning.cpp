// ============================================================================
// speed_planning.cpp — 속도 계획 함수 구현
// ============================================================================

#include "pp_controller_cpp/pursuit/speed_planning.hpp"

#include <algorithm>
#include <cmath>

namespace pp_controller_cpp
{
namespace pursuit
{

// ---------------------------------------------------------------------------
// compute_dynamic_lookahead: 속도 적응형 lookahead 거리
// ---------------------------------------------------------------------------
// 저속(커브) → 짧은 lookahead (정밀 추적)
// 고속(직선) → 긴 lookahead (안정성)
// ---------------------------------------------------------------------------
double compute_dynamic_lookahead(
  double speed,
  double lookahead_min,
  double lookahead_max,
  double speed_gain)
{
  const double unclamped = lookahead_min + speed_gain * std::max(0.0, speed);
  return std::clamp(unclamped, lookahead_min, lookahead_max);
}

// ---------------------------------------------------------------------------
// compute_speed_target: 곡률 → 목표 속도 변환
// ---------------------------------------------------------------------------
// v = sqrt(lateral_accel_limit / kappa)
// 곡률이 클수록 속도를 낮춰 횡가속도를 lateral_accel_limit 이내로 유지.
// ---------------------------------------------------------------------------
double compute_speed_target(
  double abs_kappa,
  double v_min,
  double v_max,
  double lateral_accel_limit)
{
  if (v_max <= 0.0) {
    return 0.0;
  }
  if (abs_kappa <= 1e-6) {
    return v_max;
  }

  const double curvature_limited_speed = std::sqrt(lateral_accel_limit / abs_kappa);
  return std::clamp(curvature_limited_speed, v_min, v_max);
}

// ---------------------------------------------------------------------------
// compute_path_end_speed: 남은 거리 안에 멈출 수 있는 최대 속도
// ---------------------------------------------------------------------------
// 제동거리 공식 d = v²/(2a) 를 역으로: v = √(2a × d)
// ---------------------------------------------------------------------------
double compute_path_end_speed(
  double remaining_length,
  double decel_rate,
  double v_min,
  double v_max)
{
  if (remaining_length <= 0.0 || decel_rate <= 0.0) return v_min;
  double v = std::sqrt(2.0 * decel_rate * remaining_length);
  return std::clamp(v, v_min, v_max);
}

// ---------------------------------------------------------------------------
// rate_limit_speed: 가감속 rate limit 적용
// ---------------------------------------------------------------------------
// 가속: v_cmd = min(target, prev + accel_rate * dt)
// 감속: v_cmd = max(target, prev - decel_rate * dt)
// 급격한 속도 변화를 방지하여 부드러운 주행을 보장한다.
// ---------------------------------------------------------------------------
double rate_limit_speed(
  double target_speed,
  double last_cmd_speed,
  double dt,
  double accel_rate,
  double decel_rate)
{
  if (dt <= 0.0) {
    return target_speed;
  }

  if (target_speed >= last_cmd_speed) {
    return std::min(target_speed, last_cmd_speed + accel_rate * dt);
  }
  return std::max(target_speed, last_cmd_speed - decel_rate * dt);
}

}  // namespace pursuit
}  // namespace pp_controller_cpp
