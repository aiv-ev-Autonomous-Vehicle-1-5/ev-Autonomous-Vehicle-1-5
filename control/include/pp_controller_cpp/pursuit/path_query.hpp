// ============================================================================
// path_query.hpp — 경로 분석 함수 (상태 없는 free 함수)
// ============================================================================
// 경로 점들을 분석하여 최근접점, 목표점, 잔여 거리, 곡률 등을 계산한다.
// 모든 함수는 순수 함수로, ROS2 노드에 의존하지 않는다.
//
// 함수 목록:
//   find_nearest_index        — 가장 가까운 경로점 탐색
//   compute_target_relative   — arc-length 기반 추적점 선택
//   compute_remaining_length  — 남은 경로 arc-length 계산
//   compute_preview_curvature — 전방 곡률 분석 (percentile 기반, 선감속용)
// ============================================================================

#ifndef PP_CONTROLLER_CPP__PURSUIT__PATH_QUERY_HPP_
#define PP_CONTROLLER_CPP__PURSUIT__PATH_QUERY_HPP_

#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace pp_controller_cpp
{
namespace pursuit
{

/// 경로에서 원점(0,0)에 가장 가까운 점의 인덱스를 반환
size_t find_nearest_index(const std::vector<geometry_msgs::msg::Point> & pts);

/// 상대좌표 경로에서 arc-length 기반 lookahead 목표점 계산
/// 반환: true = 유효한 목표점 발견, false = 점 부족
bool compute_target_relative(
  const std::vector<geometry_msgs::msg::Point> & pts,
  size_t nearest_i,
  double lookahead_dist,
  double & tx,
  double & ty,
  double & Ld_used);

/// nearest_i부터 경로 끝까지의 남은 arc length 계산 [m]
double compute_remaining_length(
  const std::vector<geometry_msgs::msg::Point> & pts,
  size_t nearest_i);

/// nearest_i 부터 preview_distance 까지의 구간에서 percentile 곡률 계산
/// 3점 외적 기반 곡률 추정, percentile로 노이즈 스파이크 제거 (1.0=max, 0.9=상위10% 제외)
double compute_preview_curvature(
  const std::vector<geometry_msgs::msg::Point> & pts,
  size_t nearest_i,
  double preview_distance,
  double percentile = 1.0);

}  // namespace pursuit
}  // namespace pp_controller_cpp

#endif  // PP_CONTROLLER_CPP__PURSUIT__PATH_QUERY_HPP_
