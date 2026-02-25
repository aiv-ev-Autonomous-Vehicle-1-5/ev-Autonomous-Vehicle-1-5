#include "planning_mr_ver/planner/magnetic_planner.hpp"
#include "planning_mr_ver/common/geometry.hpp"

#include <cmath>
#include <limits>

namespace planning_mr_ver
{

bool MagneticPlanner::world_to_grid(
  double wx, double wy,
  double origin_x, double origin_y,
  double resolution,
  int rows, int cols,
  int & row, int & col)
{
  col = static_cast<int>((wx - origin_x) / resolution);
  row = static_cast<int>((wy - origin_y) / resolution);
  return (row >= 0 && row < rows && col >= 0 && col < cols);
}

Point2D MagneticPlanner::grid_to_world(
  int row, int col,
  double origin_x, double origin_y,
  double resolution)
{
  return {
    origin_x + (col + 0.5) * resolution,
    origin_y + (row + 0.5) * resolution
  };
}

Point2D MagneticPlanner::find_best_forward_cell(
  const CostmapResult & costmap,
  const Point2D & current_pos,
  const Point2D & hdg,
  const PlanningParams & params,
  bool & found)
{
  found = false;
  double best_cost = std::numeric_limits<double>::max();
  double best_dot = -1.0;  // tie-breaker: heading 정렬도
  Point2D best_pos{0.0, 0.0};

  const double r = params.planner.search_radius;
  const double r_sq = r * r;
  int r_cells = static_cast<int>(std::ceil(r / costmap.resolution));

  int cur_row, cur_col;
  if (!world_to_grid(current_pos.x, current_pos.y,
      costmap.origin_x, costmap.origin_y,
      costmap.resolution, costmap.rows, costmap.cols,
      cur_row, cur_col)) {
    return best_pos;
  }

  int row_min = std::max(0, cur_row - r_cells);
  int row_max = std::min(costmap.rows - 1, cur_row + r_cells);
  int col_min = std::max(0, cur_col - r_cells);
  int col_max = std::min(costmap.cols - 1, cur_col + r_cells);

  for (int row = row_min; row <= row_max; ++row) {
    for (int col = col_min; col <= col_max; ++col) {
      Point2D cell_pos = grid_to_world(
        row, col, costmap.origin_x, costmap.origin_y, costmap.resolution);

      double dx = cell_pos.x - current_pos.x;
      double dy = cell_pos.y - current_pos.y;
      double d_sq = dx * dx + dy * dy;

      // self와 범위 밖 제외
      if (d_sq < 1e-12 || d_sq > r_sq) continue;

      // 전방 180° 체크: dot(heading, direction) > 0
      double dot_val = hdg.x * dx + hdg.y * dy;
      if (dot_val <= 0.0) continue;

      double cost = costmap.data[row * costmap.cols + col];

      // heading 방향 정렬도 (단위벡터 기준 dot, 0~1)
      double d = std::sqrt(d_sq);
      double alignment = dot_val / d;  // cos(angle)

      // 최소 cost 선택, 동률 시 heading에 더 정렬된 cell 선택 (oscillation 방지)
      if (cost < best_cost || (cost == best_cost && alignment > best_dot)) {
        best_cost = cost;
        best_dot = alignment;
        best_pos = cell_pos;
        found = true;
      }
    }
  }

  return best_pos;
}

std::vector<Point2D> MagneticPlanner::plan(
  const CostmapResult & costmap,
  const PlanningParams & params)
{
  std::vector<Point2D> raw_path;

  if (!costmap.valid) return raw_path;

  Point2D current_pos{0.0, 0.0};
  Point2D hdg = normalize(
    Point2D{params.planner.heading_init_x, params.planner.heading_init_y});

  raw_path.push_back(current_pos);

  for (int step = 0; step < params.planner.max_steps; ++step) {
    bool found = false;
    Point2D next_pos = find_best_forward_cell(
      costmap, current_pos, hdg, params, found);

    if (!found) break;

    // heading 업데이트
    Point2D new_hdg = normalize(next_pos - current_pos);
    if (norm(new_hdg) < 1e-6) break;

    hdg = new_hdg;
    current_pos = next_pos;
    raw_path.push_back(current_pos);

    // grid 경계 근처 도달 시 종료
    int r, c;
    if (!world_to_grid(current_pos.x, current_pos.y,
        costmap.origin_x, costmap.origin_y,
        costmap.resolution, costmap.rows, costmap.cols,
        r, c)) {
      break;
    }
    if (r <= 1 || r >= costmap.rows - 2 ||
        c <= 1 || c >= costmap.cols - 2) {
      break;
    }
  }

  return raw_path;
}

}  // namespace planning_mr_ver
