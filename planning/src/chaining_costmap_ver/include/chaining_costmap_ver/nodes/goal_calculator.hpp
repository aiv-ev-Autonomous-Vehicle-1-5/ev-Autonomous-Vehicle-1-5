/**
 * @file goal_calculator.hpp
 * @brief [Stage 3c] A* 탐색 목표점(local_goal) 계산 유틸리티
 *
 * on_timer()의 Stage 3에서 호출되는 goal 계산 로직을 분리한 헤더.
 * apply_center_attraction()이 생성한 centerline의 마지막 점을 goal로 사용한다.
 *
 * [구현 파일] src/nodes/goal_calculator.cpp
 */
#ifndef CHAINING_COSTMAP_VER__NODES__GOAL_CALCULATOR_HPP_
#define CHAINING_COSTMAP_VER__NODES__GOAL_CALCULATOR_HPP_

#include "chaining_costmap_ver/common/types.hpp"
#include "chaining_costmap_ver/common/params.hpp"

namespace chaining_costmap_ver
{

/**
 * @brief goal 계산 결과
 */
struct GoalResult
{
  Point2D goal{0.0, 0.0};
  bool have_goal{false};
};

/**
 * @brief costmap 월드 좌표 → 비용 조회 (범위 밖이면 999.0)
 */
double world_to_cost(
  const CostmapResult & costmap, double wx, double wy);

/**
 * @brief A* 탐색 목표점(local_goal) 계산
 *
 * 교차 상태: 해당 backbone 누적거리 중간점 사용 (기존 유지)
 * 정상 상태: center_line 마지막 점을 goal로 사용
 *   - 양쪽 backbone → midpoint 기반 centerline 끝점
 *   - 한쪽만 존재 → track_half_width 수직 오프셋 centerline 끝점
 *   - center_line이 비어있으면 goal 없음
 *
 * @param dc_result    DirectionChainer 결과
 * @param costmap      코스트맵 (clamp용)
 * @param center_line  apply_center_attraction()이 반환한 중앙선 점열
 * @param params       파라미터
 */
GoalResult calculate_goal(
  const DirectionChainResult & dc_result,
  const CostmapResult & costmap,
  const std::vector<Point2D> & center_line,
  const PlanningParams & params);

/**
 * @brief goal을 costmap 경계 안쪽으로 clamp
 *
 * backbone이 costmap보다 멀리 뻗어있으면 goal이 격자 밖에 놓여
 * A*가 즉시 빈 경로를 반환한다. 이를 방지하기 위해
 * costmap 유효 범위 안쪽 1셀 마진으로 clamp.
 */
void clamp_goal_to_costmap(
  Point2D & goal, const CostmapResult & costmap);

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__NODES__GOAL_CALCULATOR_HPP_
