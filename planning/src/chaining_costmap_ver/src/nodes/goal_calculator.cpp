/**
 * @file goal_calculator.cpp
 * @brief [Stage 3c] A* 탐색 목표점(local_goal) 계산 구현
 *
 * apply_center_attraction()이 생성한 centerline의 마지막 점을 goal로 사용한다.
 * costmap 경계 clamp도 포함.
 *
 * [교차 판정 처리]
 *   DirectionChainResult의 교차 플래그(left_crossed_right / right_crossed_left)가
 *   true인 경우, 해당 backbone의 누적거리 중간점을 local_goal로 즉시 반환한다.
 *
 * [정상 상태]
 *   center_line이 비어있지 않으면 마지막 점을 goal로 사용.
 *   비어있으면 goal 없음.
 *
 * [의존 관계]
 *   - goal_calculator.hpp: GoalResult, calculate_goal(), clamp_goal_to_costmap() 선언
 *   - types.hpp: CostmapResult, DirectionChainResult, Point2D
 *   - params.hpp: PlanningParams
 */
#include "chaining_costmap_ver/nodes/goal_calculator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace chaining_costmap_ver
{

// ============================================================================
// costmap 월드 좌표 → 비용 조회
// ============================================================================
double world_to_cost(
  const CostmapResult & costmap, double wx, double wy)
{
  if (!costmap.valid) return 999.0;
  int col = static_cast<int>((wx - costmap.origin_x) / costmap.resolution);
  int row = static_cast<int>((wy - costmap.origin_y) / costmap.resolution);
  if (row < 0 || row >= costmap.rows || col < 0 || col >= costmap.cols)
    return 999.0;
  return costmap.data[row * costmap.cols + col];
}

// ============================================================================
// A* 탐색 목표점(local_goal) 계산
// ============================================================================
// 교차 상태: backbone 누적거리 중간점 (기존 유지)
// 정상 상태: center_line 마지막 점을 goal로 사용
// ============================================================================
GoalResult calculate_goal(
  const DirectionChainResult & dc_result,
  const CostmapResult & costmap,
  const std::vector<Point2D> & center_line,
  const PlanningParams & params)
{
  (void)costmap;   // clamp는 별도 함수에서 처리
  (void)params;

  GoalResult gr;

  // 교차 판정: 한쪽 backbone이 반대쪽 seed를 먹은 경우
  // → 해당 backbone의 누적 거리 기준 중점을 local_goal로 사용
  auto midpoint_by_length = [](const std::vector<ChainPoint> & bb, Point2D & out) {
    if (bb.size() < 2) { out = {bb[0].x, bb[0].y}; return; }
    double total = 0.0;
    for (size_t i = 1; i < bb.size(); ++i) {
      total += std::hypot(bb[i].x - bb[i-1].x, bb[i].y - bb[i-1].y);
    }
    const double half = total * 0.5;
    double acc = 0.0;
    for (size_t i = 1; i < bb.size(); ++i) {
      double seg = std::hypot(bb[i].x - bb[i-1].x, bb[i].y - bb[i-1].y);
      if (acc + seg >= half) {
        double t = (seg > 1e-9) ? (half - acc) / seg : 0.0;
        out.x = bb[i-1].x + t * (bb[i].x - bb[i-1].x);
        out.y = bb[i-1].y + t * (bb[i].y - bb[i-1].y);
        return;
      }
      acc += seg;
    }
    out = {bb.back().x, bb.back().y};
  };

  if (dc_result.left_crossed_right && !dc_result.left.backbone.empty()) {
    midpoint_by_length(dc_result.left.backbone, gr.goal);
    gr.have_goal = true;
    return gr;
  }
  if (dc_result.right_crossed_left && !dc_result.right.backbone.empty()) {
    midpoint_by_length(dc_result.right.backbone, gr.goal);
    gr.have_goal = true;
    return gr;
  }

  // 정상 상태: centerline 마지막 점을 goal로 사용
  if (!center_line.empty()) {
    gr.goal = center_line.back();
    gr.have_goal = true;
  }

  return gr;
}

// ============================================================================
// goal을 costmap 경계 안쪽으로 clamp
// ============================================================================
void clamp_goal_to_costmap(
  Point2D & goal, const CostmapResult & costmap)
{
  if (!costmap.valid) return;
  const double margin = costmap.resolution;
  const double x_min = costmap.origin_x + margin;
  const double x_max = costmap.origin_x + costmap.cols * costmap.resolution - margin;
  const double y_min = costmap.origin_y + margin;
  const double y_max = costmap.origin_y + costmap.rows * costmap.resolution - margin;
  goal.x = std::clamp(goal.x, x_min, x_max);
  goal.y = std::clamp(goal.y, y_min, y_max);
}

}  // namespace chaining_costmap_ver
