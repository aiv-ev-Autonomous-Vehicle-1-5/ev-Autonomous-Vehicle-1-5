/**
 * @file costmap_generator.cpp
 * @brief Magnetic Resistance Costmap 생성기 — 구현부 (LineChainer 체인 입력 버전)
 *
 * 좌/우 체인의 각 ChainedPoint.type에 따라:
 *   - CONE → cone_cost_max + cone_radius (flat zone 있음)
 *   - LANE → lane_cost_max + inner_radius=0 (flat zone 없음)
 */
#include "planning_lc_ver/costmap/costmap_generator.hpp"

#include <cmath>
#include <algorithm>

namespace planning_lc_ver
{

double CostmapGenerator::effective_radius(
  double cost_max, double sigma, double threshold)
{
  if (threshold <= 0.0 || sigma <= 0.0) return 100.0;
  double ratio = cost_max / threshold;
  if (ratio <= 1.0) return 0.0;
  return sigma * std::sqrt(2.0 * std::log(ratio));
}

void CostmapGenerator::apply_source(
  std::vector<double> & grid,
  int rows, int cols,
  double resolution,
  double origin_x, double origin_y,
  const Point2D & source,
  double cost_max,
  double sigma,
  double threshold,
  double inner_radius)
{
  double r_decay = effective_radius(cost_max, sigma, threshold);
  double r_total = inner_radius + r_decay;
  int r_cells = static_cast<int>(std::ceil(r_total / resolution));

  const double inv_2sigma2 = -1.0 / (2.0 * sigma * sigma);

  int src_col = static_cast<int>(std::round((source.x - origin_x) / resolution));
  int src_row = static_cast<int>(std::round((source.y - origin_y) / resolution));

  int row_min = std::max(0, src_row - r_cells);
  int row_max = std::min(rows - 1, src_row + r_cells);
  int col_min = std::max(0, src_col - r_cells);
  int col_max = std::min(cols - 1, src_col + r_cells);

  for (int r = row_min; r <= row_max; ++r) {
    for (int c = col_min; c <= col_max; ++c) {
      double wx = origin_x + (c + 0.5) * resolution;
      double wy = origin_y + (r + 0.5) * resolution;

      double dx = wx - source.x;
      double dy = wy - source.y;
      double d = std::sqrt(dx * dx + dy * dy);

      double cost;
      if (d <= inner_radius) {
        cost = cost_max;
      } else {
        double d_eff = d - inner_radius;
        cost = cost_max * std::exp(inv_2sigma2 * d_eff * d_eff);
        if (cost < threshold) continue;
      }

      int idx = r * cols + c;
      if (cost > grid[idx]) {
        grid[idx] = cost;
      }
    }
  }
}

/**
 * @brief costmap 생성 — 좌/우 체인 입력
 *
 * 두 체인을 순회하며 ChainedPoint.type에 따라 적절한 파라미터로 apply_source() 호출.
 */
CostmapResult CostmapGenerator::generate(
  const std::vector<ChainedPoint> & left_chain,
  const std::vector<ChainedPoint> & right_chain,
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

  // 체인 점 적용 헬퍼 람다
  auto apply_chain = [&](const std::vector<ChainedPoint> & chain) {
    for (const auto & pt : chain) {
      const Point2D src = pt.to_point2d();
      if (pt.type == PointType::CONE) {
        apply_source(
          result.data, result.rows, result.cols,
          result.resolution, result.origin_x, result.origin_y,
          src, cm.cone_cost_max, cm.sigma, cm.cost_threshold,
          cm.cone_radius);
      } else {
        apply_source(
          result.data, result.rows, result.cols,
          result.resolution, result.origin_x, result.origin_y,
          src, cm.lane_cost_max, cm.sigma, cm.cost_threshold,
          0.0);
      }
    }
  };

  apply_chain(left_chain);
  apply_chain(right_chain);

  result.valid = true;
  return result;
}

}  // namespace planning_lc_ver
