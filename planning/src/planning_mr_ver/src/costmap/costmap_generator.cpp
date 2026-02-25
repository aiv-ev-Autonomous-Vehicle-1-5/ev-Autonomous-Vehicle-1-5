#include "planning_mr_ver/costmap/costmap_generator.hpp"

#include <cmath>
#include <algorithm>

namespace planning_mr_ver
{

double CostmapGenerator::effective_radius(
  double cost_max, double alpha, double threshold)
{
  if (threshold <= 0.0 || alpha <= 0.0) return 100.0;
  double ratio = cost_max / threshold;
  if (ratio <= 1.0) return 0.0;
  return std::sqrt((ratio - 1.0) / alpha);
}

void CostmapGenerator::apply_source(
  std::vector<double> & grid,
  int rows, int cols,
  double resolution,
  double origin_x, double origin_y,
  const Point2D & source,
  double cost_max,
  double alpha,
  double threshold,
  double inner_radius)
{
  // 총 영향 반경 = inner_radius (flat zone) + decay의 effective_radius
  double r_decay = effective_radius(cost_max, alpha, threshold);
  double r_total = inner_radius + r_decay;
  int r_cells = static_cast<int>(std::ceil(r_total / resolution));

  // source를 grid 좌표로 변환
  int src_col = static_cast<int>(std::round((source.x - origin_x) / resolution));
  int src_row = static_cast<int>(std::round((source.y - origin_y) / resolution));

  // bounding box 제한
  int row_min = std::max(0, src_row - r_cells);
  int row_max = std::min(rows - 1, src_row + r_cells);
  int col_min = std::max(0, src_col - r_cells);
  int col_max = std::min(cols - 1, src_col + r_cells);

  for (int r = row_min; r <= row_max; ++r) {
    for (int c = col_min; c <= col_max; ++c) {
      // cell 중심의 world 좌표
      double wx = origin_x + (c + 0.5) * resolution;
      double wy = origin_y + (r + 0.5) * resolution;

      double dx = wx - source.x;
      double dy = wy - source.y;
      double d = std::sqrt(dx * dx + dy * dy);

      double cost;
      if (d <= inner_radius) {
        // flat zone: 클러스터 반지름 내 → 최대 cost
        cost = cost_max;
      } else {
        // decay zone: 반지름 밖부터 1/r² 감쇠
        double d_eff = d - inner_radius;
        cost = cost_max / (1.0 + alpha * d_eff * d_eff);
        if (cost < threshold) continue;
      }

      // MAX override
      int idx = r * cols + c;
      if (cost > grid[idx]) {
        grid[idx] = cost;
      }
    }
  }
}

CostmapResult CostmapGenerator::generate(
  const std::vector<Point2D> & cones,
  const std::vector<Point2D> & lanes,
  const PlanningParams & params)
{
  CostmapResult result;

  const auto & cm = params.costmap;
  result.resolution = cm.resolution;
  result.cols = static_cast<int>(std::round(cm.size_x / cm.resolution));
  result.rows = static_cast<int>(std::round(cm.size_y / cm.resolution));
  result.origin_x = -cm.size_x / 2.0;
  result.origin_y = -cm.size_y / 2.0;

  result.data.assign(result.rows * result.cols, 0.0);

  // 콘: inner_radius = cone_radius (flat zone 있음)
  for (const auto & cone : cones) {
    apply_source(
      result.data, result.rows, result.cols,
      result.resolution, result.origin_x, result.origin_y,
      cone, cm.cone_cost_max, cm.alpha, cm.cost_threshold,
      cm.cone_radius);
  }

  // 차선: inner_radius = 0 (두께 없음, 중심부터 바로 감쇠)
  for (const auto & lane_pt : lanes) {
    apply_source(
      result.data, result.rows, result.cols,
      result.resolution, result.origin_x, result.origin_y,
      lane_pt, cm.lane_cost_max, cm.alpha, cm.cost_threshold,
      0.0);
  }

  result.valid = true;
  return result;
}

}  // namespace planning_mr_ver
