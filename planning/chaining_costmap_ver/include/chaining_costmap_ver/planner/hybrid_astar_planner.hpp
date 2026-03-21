/**
 * @file hybrid_astar_planner.hpp
 * @brief Hybrid A* 경로 탐색기
 *
 * (x, y, θ) 상태 공간에서 차량 운동학(bicycle model)을 반영하여 경로를 탐색한다.
 * 기존 AStarPlanner와 동일한 CostmapResult 입력을 사용하므로 두 버전을 직접 비교 가능.
 *
 * ── 기존 A*와의 차이점 ──
 *   A*        : (row, col) 격자 셀만 고려. 헤딩 무관. 8방향 확장.
 *   Hybrid A* : (x, y, θ) 연속 상태. 차량 조향각 기반 확장(bicycle model).
 *               생성된 경로가 차량이 실제로 주행 가능한 곡선으로 구성됨.
 *
 * ── 알고리즘 개요 ──
 *   상태 공간 : (x, y, yaw) — 연속 좌표 + 방향각
 *   모션 프리미티브: N_STEER(7)개 조향각 × arc_length(0.3m) 호 이동
 *   운동학 모델: 자전거 모델 (bicycle model, midpoint rule)
 *     kappa   = tan(δ) / L
 *     mid_yaw = yaw + kappa * arc_len / 2
 *     x'      = x + arc_len * cos(mid_yaw)
 *     y'      = y + arc_len * sin(mid_yaw)
 *     yaw'    = yaw + kappa * arc_len
 *   비용 함수 : g += arc_len + costmap_cost * cost_weight
 *   휴리스틱  : 유클리드 거리 (non-admissible but practical)
 *   방문 체크 : (col, row, yaw_bin) 삼중 키 — unordered_map<int, double>
 *               yaw_bin = 5° 단위 이산화 (N_YAW = 72)
 *
 * ── 파라미터 재사용 ──
 *   max_iterations / goal_tolerance / cost_weight / obstacle_cost → params.astar
 *   wheelbase / delta_max                                         → params.vehicle
 *   arc_length / enabled                                          → params.hybrid_astar
 */
#ifndef CHAINING_COSTMAP_VER__PLANNER__HYBRID_ASTAR_PLANNER_HPP_
#define CHAINING_COSTMAP_VER__PLANNER__HYBRID_ASTAR_PLANNER_HPP_

#include "chaining_costmap_ver/common/types.hpp"
#include "chaining_costmap_ver/common/params.hpp"

#include <vector>

namespace chaining_costmap_ver
{

class HybridAStarPlanner
{
public:
  /**
   * @brief Hybrid A* 경로 탐색
   *
   * 기존 AStarPlanner::plan()과 동일한 시그니처 — 노드에서 플래너만 교체 가능.
   *
   * @param costmap  CostmapGenerator 출력 (기존 A*와 동일한 입력)
   * @param start    시작점 (base_link 기준, 보통 {0, 0})
   * @param goal     목표점 (backbone 끝점 중점)
   * @param params   플래닝 파라미터 (astar / vehicle / hybrid_astar 사용)
   * @return 경로 Point2D[] (start→goal 순서). 탐색 실패 시 빈 벡터.
   */
  std::vector<Point2D> plan(
    const CostmapResult & costmap,
    const Point2D & start,
    const Point2D & goal,
    const PlanningParams & params) const;
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__PLANNER__HYBRID_ASTAR_PLANNER_HPP_
