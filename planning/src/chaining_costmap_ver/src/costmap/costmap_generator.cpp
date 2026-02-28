/**
 * @file costmap_generator.cpp
 * @brief 가우시안 비용 지도(Costmap) 생성기 — 구현부
 *
 * 좌/우 경계 체인의 각 점(ChainedPoint)에서 가우시안 비용장을 방사하여
 * 2D 격자 비용 지도를 생성한다.
 *
 * CONE: cone_cost_max + cone_radius flat zone + 가우시안 감쇠
 * LANE: lane_cost_max + flat zone 없음 + 가우시안 감쇠
 * unchained: CONE 비용으로 보수적 처리 (체이닝 실패 = 미확인 장애물)
 */
#include "chaining_costmap_ver/costmap/costmap_generator.hpp"

#include <cmath>
#include <algorithm>

namespace chaining_costmap_ver
{

// ============================================================================
// effective_radius
// ============================================================================
double CostmapGenerator::effective_radius(
  double cost_max, double sigma, double threshold)
{
  if (threshold <= 0.0 || sigma <= 0.0) return 100.0;
  double ratio = cost_max / threshold;
  if (ratio <= 1.0) return 0.0;
  return sigma * std::sqrt(2.0 * std::log(ratio));
}

// ============================================================================
// apply_source
// ============================================================================
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
  // 유효 반경 계산 → source 주변만 순회
  double r_decay = effective_radius(cost_max, sigma, threshold);
  double r_total = inner_radius + r_decay;
  int r_cells = static_cast<int>(std::ceil(r_total / resolution));

  const double inv_2sigma2 = -1.0 / (2.0 * sigma * sigma);

  // source의 그리드 좌표
  int src_col = static_cast<int>(std::round((source.x - origin_x) / resolution));
  int src_row = static_cast<int>(std::round((source.y - origin_y) / resolution));

  // 순회 범위 클리핑
  int row_min = std::max(0, src_row - r_cells);
  int row_max = std::min(rows - 1, src_row + r_cells);
  int col_min = std::max(0, src_col - r_cells);
  int col_max = std::min(cols - 1, src_col + r_cells);

  for (int r = row_min; r <= row_max; ++r) {
    for (int c = col_min; c <= col_max; ++c) {
      // 셀 중심의 월드 좌표
      double wx = origin_x + (c + 0.5) * resolution;
      double wy = origin_y + (r + 0.5) * resolution;

      double dx = wx - source.x;
      double dy = wy - source.y;
      double d = std::sqrt(dx * dx + dy * dy);

      double cost;
      if (d <= inner_radius) {
        // flat zone: 콘의 물리적 크기 내 → 최대 비용
        cost = cost_max;
      } else {
        // 가우시안 감쇠
        double d_eff = d - inner_radius;
        cost = cost_max * std::exp(inv_2sigma2 * d_eff * d_eff);
        if (cost < threshold) continue;
      }

      // max-merge: 여러 source가 겹치면 큰 값 유지
      int idx = r * cols + c;
      if (cost > grid[idx]) {
        grid[idx] = cost;
      }
    }
  }
}

// ============================================================================
// generate (2-chain version)
// ============================================================================
CostmapResult CostmapGenerator::generate(
  const std::vector<ChainedPoint> & left_chain,
  const std::vector<ChainedPoint> & right_chain,
  const PlanningParams & params)
{
  // unchained 없는 버전: 빈 벡터로 3-arg 버전 호출
  return generate(left_chain, right_chain, {}, params);
}

// ============================================================================
// generate (3-chain version, with unchained)
// ============================================================================
CostmapResult CostmapGenerator::generate(
  const std::vector<ChainedPoint> & left_chain,
  const std::vector<ChainedPoint> & right_chain,
  const std::vector<ChainedPoint> & unchained,
  const PlanningParams & params)
{
  CostmapResult result;

  const auto & cm = params.costmap;
  result.resolution = cm.resolution;
  result.cols = static_cast<int>(std::round(cm.size_x / cm.resolution));
  result.rows = static_cast<int>(std::round(cm.size_y / cm.resolution));
  result.origin_x = 0;
  result.origin_y = -cm.size_y / 2.0;

  // 전체 그리드를 0.0(자유 공간)으로 초기화
  result.data.assign(result.rows * result.cols, 0.0);

  // 체인 포인트 비용 적용 (CONE/LANE 분기)
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

  // unchained 포인트: 체이닝 실패 = 미확인 장애물 → CONE 비용으로 보수적 처리
  for (const auto & pt : unchained) {
    const Point2D src = pt.to_point2d();
    apply_source(
      result.data, result.rows, result.cols,
      result.resolution, result.origin_x, result.origin_y,
      src, cm.cone_cost_max, cm.sigma, cm.cost_threshold,
      cm.cone_radius);
  }

  result.valid = true;
  return result;
}

}  // namespace chaining_costmap_ver
