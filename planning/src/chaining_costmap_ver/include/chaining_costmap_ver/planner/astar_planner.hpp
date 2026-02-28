/**
 * @file astar_planner.hpp
 * @brief A* 격자 경로 탐색기
 *
 * CostmapGenerator가 생성한 2D 비용 지도 위에서
 * 8방향 A* 알고리즘으로 최적 경로를 탐색한다.
 *
 * [알고리즘]
 *   - f(n) = g(n) + h(n)
 *   - g(n) = 누적 이동비용 + costmap_cost × cost_weight
 *   - h(n) = 유클리드 거리 (admissible heuristic)
 *   - 8방향 확장 (cardinal=1.0, diagonal=√2)
 *   - cost >= obstacle_cost 인 셀은 통과 불가
 *
 * [입출력]
 *   입력: CostmapResult + start(ego={0,0}) + goal(chain 끝점 중점)
 *   출력: Point2D[] (world 좌표 경로)
 */
#ifndef CHAINING_COSTMAP_VER__PLANNER__ASTAR_PLANNER_HPP_
#define CHAINING_COSTMAP_VER__PLANNER__ASTAR_PLANNER_HPP_

#include "chaining_costmap_ver/common/types.hpp"
#include "chaining_costmap_ver/common/params.hpp"
#include <vector>

namespace chaining_costmap_ver
{

class AStarPlanner
{
public:
  /**
   * @brief A* 경로 탐색
   *
   * @param costmap  CostmapGenerator 출력 (2D 비용 격자)
   * @param start    시작점 (ego, 보통 {0,0})
   * @param goal     목표점 (chain 끝점 중점)
   * @param params   A* 파라미터 (max_iterations, goal_tolerance 등)
   * @return 경로 Point2D[] (start→goal 순서). 탐색 실패 시 빈 벡터.
   */
  std::vector<Point2D> plan(
    const CostmapResult & costmap,
    const Point2D & start,
    const Point2D & goal,
    const PlanningParams & params) const;
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__PLANNER__ASTAR_PLANNER_HPP_
