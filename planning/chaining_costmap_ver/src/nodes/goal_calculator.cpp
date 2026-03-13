/**
 * @file goal_calculator.cpp
 * @brief [Stage 3c] A* 탐색 목표점(local_goal) 계산 구현
 *
 * 좌/우 backbone 끝점을 잇는 선분 위에서 통과 가능한 goal을 결정한다.
 * costmap 경계 clamp도 포함.
 *
 * [의존 관계]
 *   - goal_calculator.hpp: GoalResult, calculate_goal(), clamp_goal_to_costmap() 선언
 *   - types.hpp: CostmapResult, DirectionChainResult, Point2D
 *   - params.hpp: PlanningParams::AStar (goal_max_cost)
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
GoalResult calculate_goal(
  const DirectionChainResult & dc_result,
  const CostmapResult & costmap,
  const PlanningParams & params)
{
  GoalResult gr;

  if (!dc_result.left.backbone.empty() && !dc_result.right.backbone.empty()) {
    const double lx = dc_result.left.backbone.back().x;
    const double rx = dc_result.right.backbone.back().x;
    const double ly = dc_result.left.backbone.back().y;
    const double ry = dc_result.right.backbone.back().y;

    // 선분 중점
    const double mx = (lx + rx) / 2.0;
    const double my = (ly + ry) / 2.0;

    const double goal_max_cost = params.astar.goal_max_cost;

    // 중점의 cost가 goal_max_cost 미만이면 바로 사용
    if (world_to_cost(costmap, mx, my) < goal_max_cost) {
      gr.goal.x = mx;
      gr.goal.y = my;
    } else {
      // 선분 위 픽셀을 resolution 간격으로 샘플링하여
      // cost < goal_max_cost인 점 중 중점에 가장 가까운 점 탐색
      const double seg_dx = rx - lx;
      const double seg_dy = ry - ly;
      const double seg_len = std::hypot(seg_dx, seg_dy);
      const double step = costmap.valid ? costmap.resolution : 0.05;
      const int n_samples = std::max(2, static_cast<int>(seg_len / step) + 1);

      double best_dist2 = std::numeric_limits<double>::infinity();
      bool found = false;

      for (int i = 0; i <= n_samples; ++i) {
        const double t = static_cast<double>(i) / n_samples;
        const double sx = lx + seg_dx * t;
        const double sy = ly + seg_dy * t;
        if (world_to_cost(costmap, sx, sy) < goal_max_cost) {
          const double d2 = (sx - mx) * (sx - mx) + (sy - my) * (sy - my);
          if (d2 < best_dist2) {
            best_dist2 = d2;
            gr.goal.x = sx;
            gr.goal.y = sy;
            found = true;
          }
        }
      }
      // 선분 위에 free 셀이 없으면 중점을 폴백으로 사용
      if (!found) {
        gr.goal.x = mx;
        gr.goal.y = my;
      }
    }
    gr.have_goal = true;
  } else if (!dc_result.left.backbone.empty()) {
    gr.goal.x = dc_result.left.backbone.back().x;
    gr.goal.y = dc_result.left.backbone.back().y * 0.5;
    gr.have_goal = true;
  } else if (!dc_result.right.backbone.empty()) {
    gr.goal.x = dc_result.right.backbone.back().x;
    gr.goal.y = dc_result.right.backbone.back().y * 0.5;
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
