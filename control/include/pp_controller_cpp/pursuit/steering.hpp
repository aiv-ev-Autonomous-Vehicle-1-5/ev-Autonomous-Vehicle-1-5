// ============================================================================
// steering.hpp — Pure Pursuit 조향각 계산 (상태 없는 free 함수)
// ============================================================================
// Pure Pursuit 핵심 공식에 의한 조향각 계산만 담당한다.
// ============================================================================

#ifndef PP_CONTROLLER_CPP__PURSUIT__STEERING_HPP_
#define PP_CONTROLLER_CPP__PURSUIT__STEERING_HPP_

namespace pp_controller_cpp
{
namespace pursuit
{

/// Pure Pursuit 조향각 계산
/// kappa = 2 * ty / Ld^2,  delta = clamp(atan(L * kappa), ±delta_max)
double compute_steering(
  double ty,
  double Ld_used,
  double wheelbase,
  double delta_max);

}  // namespace pursuit
}  // namespace pp_controller_cpp

#endif  // PP_CONTROLLER_CPP__PURSUIT__STEERING_HPP_
