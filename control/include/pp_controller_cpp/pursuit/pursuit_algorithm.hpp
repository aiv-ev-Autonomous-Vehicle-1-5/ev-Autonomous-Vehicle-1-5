// ============================================================================
// pursuit_algorithm.hpp — Pure Pursuit 핵심 알고리즘 (상태 없는 free 함수)
// ============================================================================
// 순수 계산 함수만 포함하며, ROS2 노드에 의존하지 않는다.
// 모든 함수는 입력을 받아 출력을 반환하는 순수 함수로,
// 독립적으로 테스트하기 용이하다.
//
// 함수 목록:
//   find_nearest_index        — 가장 가까운 경로점 탐색
//   compute_dynamic_lookahead — 속도 적응형 lookahead 거리 계산
//   compute_preview_curvature — 전방 곡률 분석 (선감속용)
//   compute_speed_target      — 곡률 → 속도 변환
//   compute_target_relative   — arc-length 기반 추적점 선택
//   rate_limit_speed          — 가감속 제한
//   compute_steering          — Pure Pursuit 조향각 계산
// ============================================================================

#ifndef PP_CONTROLLER_CPP__PURSUIT__PURSUIT_ALGORITHM_HPP_
#define PP_CONTROLLER_CPP__PURSUIT__PURSUIT_ALGORITHM_HPP_

#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace pp_controller_cpp
{
namespace pursuit
{

/// 경로에서 원점(0,0)에 가장 가까운 점의 인덱스를 반환
size_t find_nearest_index(const std::vector<geometry_msgs::msg::Point> & pts);

/// 속도에 따른 동적 lookahead 거리 계산
/// Ld = clamp(min + gain * speed, min, max)
double compute_dynamic_lookahead(
  double speed,
  double lookahead_min,
  double lookahead_max,
  double speed_gain);

/// nearest_i 부터 preview_distance 까지의 구간에서 최대 곡률 계산
/// 3점 외적 기반 곡률 추정으로 선감속에 사용
double compute_preview_curvature(
  const std::vector<geometry_msgs::msg::Point> & pts,
  size_t nearest_i,
  double preview_distance);

/// 곡률 → 목표 속도 변환
/// v = clamp(sqrt(lateral_accel_limit / kappa), v_min, v_max)
double compute_speed_target(
  double abs_kappa,
  double v_min,
  double v_max,
  double lateral_accel_limit);

/// 상대좌표 경로에서 arc-length 기반 lookahead 목표점 계산
/// 반환: true = 유효한 목표점 발견, false = 점 부족
bool compute_target_relative(
  const std::vector<geometry_msgs::msg::Point> & pts,
  size_t nearest_i,
  double lookahead_dist,
  double & tx,
  double & ty,
  double & Ld_used);

/// 가감속 rate limit 적용
double rate_limit_speed(
  double target_speed,
  double last_cmd_speed,
  double dt,
  double accel_rate,
  double decel_rate);

/// Pure Pursuit 조향각 계산
/// kappa = 2 * ty / Ld^2,  delta = clamp(atan(L * kappa), ±delta_max)
double compute_steering(
  double ty,
  double Ld_used,
  double wheelbase,
  double delta_max);

}  // namespace pursuit
}  // namespace pp_controller_cpp

#endif  // PP_CONTROLLER_CPP__PURSUIT__PURSUIT_ALGORITHM_HPP_
