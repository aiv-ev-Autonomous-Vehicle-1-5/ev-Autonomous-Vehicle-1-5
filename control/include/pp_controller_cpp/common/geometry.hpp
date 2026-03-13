// ============================================================================
// geometry.hpp — 2D 기하학 유틸리티 (header-only)
// ============================================================================
// Pure Pursuit 알고리즘에서 사용하는 기본 기하학 연산.
//
// 용도:
//   1) path 상의 연속된 두 점 사이의 거리 (누적 arc length 계산)
//   2) 차량 원점에서 목표점까지의 직선 거리 Ld 계산
// ============================================================================

#ifndef PP_CONTROLLER_CPP__COMMON__GEOMETRY_HPP_
#define PP_CONTROLLER_CPP__COMMON__GEOMETRY_HPP_

#include <cmath>

namespace pp_controller_cpp
{

/// 2차원 유클리드 거리 계산
/// 예: norm2d(3, 4) = 5.0
inline double norm2d(double x, double y)
{
  return std::sqrt(x * x + y * y);
}

}  // namespace pp_controller_cpp

#endif  // PP_CONTROLLER_CPP__COMMON__GEOMETRY_HPP_
