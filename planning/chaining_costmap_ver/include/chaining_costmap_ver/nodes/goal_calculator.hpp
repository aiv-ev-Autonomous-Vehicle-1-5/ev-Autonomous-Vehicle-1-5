/**
 * @file goal_calculator.hpp
 * @brief [Stage 3c] A* 탐색 목표점(local_goal) 계산 유틸리티
 *
 * on_timer()의 Stage 3에서 호출되는 goal 계산 로직을 분리한 헤더.
 * 좌/우 backbone 끝점을 잇는 선분 위에서 통과 가능한 goal을 결정한다.
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
 * Case 1: 양쪽 backbone 모두 존재
 *   1) 좌/우 끝점을 잇는 선분의 중점 cost가 goal_max_cost 미만 → 중점 사용
 *   2) 중점이 장애물 → 선분 위에서 cost < goal_max_cost인 점 중 중점에 가장 가까운 점
 *
 * Case 2/3: 한쪽만 존재
 *   goal.x = 해당 끝점.x,  goal.y = 해당 끝점.y × 0.5 (중앙 쪽으로 보정)
 *
 * @param dc_result  DirectionChainer 결과
 * @param costmap    코스트맵 (goal 통과 가능 여부 판별용)
 * @param params     파라미터 (goal_max_cost)
 */
GoalResult calculate_goal(
  const DirectionChainResult & dc_result,
  const CostmapResult & costmap,
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
