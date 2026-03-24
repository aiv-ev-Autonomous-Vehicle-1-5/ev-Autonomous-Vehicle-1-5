// ============================================================================
// speed_planning.hpp — 속도 계획 함수 (상태 없는 free 함수)
// ============================================================================
// 속도 관련 순수 계산 함수만 포함하며, ROS2 노드에 의존하지 않는다.
// 곡률/제동거리 기반 속도 결정 및 가감속 제한을 담당한다.
//
// 함수 목록:
//   compute_dynamic_lookahead — 속도 적응형 lookahead 거리 계산
//   compute_speed_target      — 곡률 → 속도 변환
//   compute_path_end_speed    — 제동거리 기반 최대 속도
//   rate_limit_speed          — 가감속 제한
// ============================================================================

#ifndef PP_CONTROLLER_CPP__PURSUIT__SPEED_PLANNING_HPP_
#define PP_CONTROLLER_CPP__PURSUIT__SPEED_PLANNING_HPP_

namespace pp_controller_cpp
{
namespace pursuit
{

/// 속도에 따른 동적 lookahead 거리 계산
/// Ld = clamp(min + gain * speed, min, max)
double compute_dynamic_lookahead(
  double speed,
  double lookahead_min,
  double lookahead_max,
  double speed_gain);

/// 곡률 → 목표 속도 변환
/// v = clamp(sqrt(lateral_accel_limit / kappa), v_min, v_max)
double compute_speed_target(
  double abs_kappa,
  double v_min,
  double v_max,
  double lateral_accel_limit);

/// 남은 경로 길이 안에 멈출 수 있는 최대 속도 계산
/// v = sqrt(2 * decel_rate * remaining_length)
double compute_path_end_speed(
  double remaining_length,
  double decel_rate,
  double v_min,
  double v_max);

/// 가감속 rate limit 적용
double rate_limit_speed(
  double target_speed,
  double last_cmd_speed,
  double dt,
  double accel_rate,
  double decel_rate);

}  // namespace pursuit
}  // namespace pp_controller_cpp

#endif  // PP_CONTROLLER_CPP__PURSUIT__SPEED_PLANNING_HPP_
