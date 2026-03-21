// ============================================================================
// steering.cpp — Pure Pursuit 조향각 계산 구현
// ============================================================================

#include "pp_controller_cpp/pursuit/steering.hpp"

#include <algorithm>
#include <cmath>

namespace pp_controller_cpp
{
namespace pursuit
{

// ---------------------------------------------------------------------------
// compute_steering: Pure Pursuit 조향각 계산
// ---------------------------------------------------------------------------
// 핵심 공식:
//   kappa = 2 * ty / Ld^2        (곡률)
//   delta = atan(L * kappa)      (조향각)
//   delta = clamp(delta, ±delta_max)  (하드웨어 제한)
//
// ty > 0 → 좌회전 (delta > 0)
// ty < 0 → 우회전 (delta < 0)
// ---------------------------------------------------------------------------
double compute_steering(
  double ty,
  double Ld_used,
  double wheelbase,
  double delta_max)
{
  const double kappa = (2.0 * ty) / (Ld_used * Ld_used);
  double delta = std::atan(wheelbase * kappa);
  return std::clamp(delta, -delta_max, delta_max);
}

}  // namespace pursuit
}  // namespace pp_controller_cpp
