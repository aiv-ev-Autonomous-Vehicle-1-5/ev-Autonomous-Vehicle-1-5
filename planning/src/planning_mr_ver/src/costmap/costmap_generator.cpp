/**
 * @file costmap_generator.cpp
 * @brief Magnetic Resistance Costmap 생성기 — 구현부
 *
 * 각 장애물(콘, 차선)을 S극 자석으로 모델링하여
 * ego 중심 그리드에 Gaussian 감쇠 cost를 기록한다.
 *
 * 감쇠 모델 (nav2 costmap_2d 방식):
 *   cost = cost_max · exp(-d_eff² / (2σ²))
 *   d_eff = max(0, d - inner_radius)
 *   유효 반경 ≈ σ · sqrt(2·ln(cost_max/threshold))
 *
 * Gaussian vs Lorentzian(이전):
 *   Gaussian: 3σ 밖은 깨끗하게 0 → 원거리 간섭 없음
 *   Lorentz:  꼬리가 무거워 5m에서도 cost=2 잔류 → local minima 유발
 *
 * 전체 흐름:
 *   1. 빈 그리드 초기화 (모든 셀 = 0)
 *   2. 각 콘에 대해 apply_source() (flat zone + Gaussian decay)
 *   3. 각 차선 점에 대해 apply_source() (Gaussian decay만)
 *   4. 완성된 costmap 반환
 */
#include "planning_mr_ver/costmap/costmap_generator.hpp"

#include <cmath>
#include <algorithm>

namespace planning_mr_ver
{

/**
 * @brief Gaussian decay zone의 유효 영향 반경 계산
 *
 * cost_max · exp(-r² / (2σ²)) = threshold 를 r에 대해 풀면:
 *   r = σ · sqrt(2 · ln(cost_max / threshold))
 *
 * 예: cost_max=100, σ=1.0, threshold=2.0
 *     r = 1.0 · sqrt(2 · ln(50)) = sqrt(7.82) ≈ 2.80m
 *
 * 비교: 이전 Lorentzian(alpha=1.5)은 r≈5.7m → Gaussian이 훨씬 깔끔
 *
 * @return decay zone만의 유효 반경 [m]
 */
double CostmapGenerator::effective_radius(
  double cost_max, double sigma, double threshold)
{
  if (threshold <= 0.0 || sigma <= 0.0) return 100.0;
  double ratio = cost_max / threshold;
  if (ratio <= 1.0) return 0.0;
  // r = σ · sqrt(2 · ln(ratio))
  return sigma * std::sqrt(2.0 * std::log(ratio));
}

/**
 * @brief 단일 source의 자력을 grid에 적용
 *
 * 처리 단계:
 *   1. 총 영향 반경 계산 = inner_radius + effective_radius
 *   2. source의 그리드 좌표 변환
 *   3. 영향 범위(bounding box)의 셀만 순회 (전체 그리드 순회 방지)
 *   4. 각 셀의 월드 좌표로부터 source까지의 거리 계산
 *   5. flat zone / decay zone 판정 → cost 계산
 *   6. 기존 값보다 큰 경우에만 덮어쓰기 (MAX override)
 */
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
  // 총 영향 반경 = flat zone + Gaussian decay가 threshold에 도달하는 거리
  double r_decay = effective_radius(cost_max, sigma, threshold);
  double r_total = inner_radius + r_decay;
  int r_cells = static_cast<int>(std::ceil(r_total / resolution));

  // Gaussian 계산용 상수: -1 / (2σ²)  → exp(inv_2sigma2 * d²) 형태로 사용
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
        // ── flat zone ──
        // 콘의 물리적 반지름 내부 → 최대 cost (충돌 영역)
        cost = cost_max;
      } else {
        // ── Gaussian decay zone ──
        // cost = cost_max · exp(-d_eff² / (2σ²))
        // Lorentzian 대비 장점: 3σ 밖은 깨끗하게 0, 원거리 간섭 없음
        double d_eff = d - inner_radius;
        cost = cost_max * std::exp(inv_2sigma2 * d_eff * d_eff);
        if (cost < threshold) continue;
      }

      // MAX override: 여러 source 중 가장 강한 cost만 유지
      int idx = r * cols + c;
      if (cost > grid[idx]) {
        grid[idx] = cost;
      }
    }
  }
}

/**
 * @brief costmap 생성 메인 함수
 *
 * 처리 순서:
 *   1. 파라미터로부터 그리드 크기 계산 (cols = size_x/resolution, rows = size_y/resolution)
 *   2. origin 설정: ego(0,0)가 그리드 중심이 되도록 (-size/2, -size/2)
 *   3. 빈 그리드 초기화 (전체 0.0)
 *   4. 콘 적용: inner_radius = cone_radius (flat zone 있음, PE 드럼 반지름)
 *   5. 차선 적용: inner_radius = 0 (두께 없음, 중심부터 바로 감쇠)
 *   6. valid = true 설정 후 반환
 */
CostmapResult CostmapGenerator::generate(
  const std::vector<Point2D> & cones,
  const std::vector<Point2D> & lanes,
  const PlanningParams & params)
{
  CostmapResult result;

  const auto & cm = params.costmap;
  result.resolution = cm.resolution;
  // 그리드 크기: 16m / 0.05m = 400 cells
  result.cols = static_cast<int>(std::round(cm.size_x / cm.resolution));
  result.rows = static_cast<int>(std::round(cm.size_y / cm.resolution));
  // origin: ego가 그리드 중심이 되도록 좌하단 좌표 설정
  result.origin_x = -cm.size_x / 2.0 ; 
  result.origin_y = -cm.size_y / 2.0;  

  // 전체 그리드를 0으로 초기화 (장애물 없는 빈 공간)
  result.data.assign(result.rows * result.cols, 0.0);

  // ── 콘 적용 ──
  // inner_radius = cone_radius (0.65m): PE 드럼의 물리적 반지름
  // 이 내부는 cost_max(100)로 채워짐 → 절대 진입 불가 영역
  for (const auto & cone : cones) {
    apply_source(
      result.data, result.rows, result.cols,
      result.resolution, result.origin_x, result.origin_y,
      cone, cm.cone_cost_max, cm.sigma, cm.cost_threshold,
      cm.cone_radius);
  }

  // ── 차선 적용 ──
  // inner_radius = 0: 차선은 두께가 없으므로 flat zone 없음
  // 중심(차선 위)부터 바로 감쇠 시작, cost_max = 50 (콘보다 약함)
  for (const auto & lane_pt : lanes) {
    apply_source(
      result.data, result.rows, result.cols,
      result.resolution, result.origin_x, result.origin_y,
      lane_pt, cm.lane_cost_max, cm.sigma, cm.cost_threshold,
      0.0);  // inner_radius = 0 (두께 없음)
  }

  result.valid = true;
  return result;
}

}  // namespace planning_mr_ver
