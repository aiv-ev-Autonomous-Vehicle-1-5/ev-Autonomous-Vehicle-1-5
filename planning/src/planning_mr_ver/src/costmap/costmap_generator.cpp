/**
 * @file costmap_generator.cpp
 * @brief Magnetic Resistance Costmap 생성기 — 구현부
 *
 * 각 장애물(콘, 차선)을 S극 자석으로 모델링하여
 * ego 중심 10×10m 그리드에 자력(cost)을 기록한다.
 *
 * 전체 흐름:
 *   1. 빈 그리드 초기화 (모든 셀 = 0)
 *   2. 각 콘에 대해 apply_source() (flat zone + decay zone)
 *   3. 각 차선 점에 대해 apply_source() (decay zone만)
 *   4. 완성된 costmap 반환
 */
#include "planning_mr_ver/costmap/costmap_generator.hpp"

#include <cmath>
#include <algorithm>

namespace planning_mr_ver
{

/**
 * @brief decay zone의 유효 영향 반경 계산
 *
 * cost_max / (1 + alpha * r²) = threshold 를 r에 대해 풀면:
 *   r = sqrt((cost_max / threshold - 1) / alpha)
 *
 * 예: cost_max=100, alpha=2.0, threshold=2.0
 *     r = sqrt((100/2 - 1) / 2) = sqrt(49/2) ≈ 4.95m
 *
 * @return decay zone만의 유효 반경 [m]
 */
double CostmapGenerator::effective_radius(
  double cost_max, double alpha, double threshold)
{
  // 예외 처리: threshold나 alpha가 0 이하면 매우 넓은 범위 반환
  if (threshold <= 0.0 || alpha <= 0.0) return 100.0;
  double ratio = cost_max / threshold;
  // cost_max이 threshold 이하면 어떤 거리에서도 영향 없음
  if (ratio <= 1.0) return 0.0;
  return std::sqrt((ratio - 1.0) / alpha);
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
  double alpha,
  double threshold,
  double inner_radius)
{
  // 총 영향 반경 = flat zone 크기 + decay가 threshold에 도달하는 거리
  double r_decay = effective_radius(cost_max, alpha, threshold);
  double r_total = inner_radius + r_decay;
  // 영향 범위를 셀 단위로 변환 (올림하여 여유 확보)
  int r_cells = static_cast<int>(std::ceil(r_total / resolution));

  // source 위치를 그리드 인덱스(행, 열)로 변환
  int src_col = static_cast<int>(std::round((source.x - origin_x) / resolution));
  int src_row = static_cast<int>(std::round((source.y - origin_y) / resolution));

  // 순회 범위를 그리드 경계 내로 제한 (bounding box clamp)
  int row_min = std::max(0, src_row - r_cells);
  int row_max = std::min(rows - 1, src_row + r_cells);
  int col_min = std::max(0, src_col - r_cells);
  int col_max = std::min(cols - 1, src_col + r_cells);

  for (int r = row_min; r <= row_max; ++r) {
    for (int c = col_min; c <= col_max; ++c) {
      // 셀 중심의 월드 좌표 (+0.5: 셀 중심 보정)
      double wx = origin_x + (c + 0.5) * resolution;
      double wy = origin_y + (r + 0.5) * resolution;

      // source까지의 유클리드 거리
      double dx = wx - source.x;
      double dy = wy - source.y;
      double d = std::sqrt(dx * dx + dy * dy);

      double cost;
      if (d <= inner_radius) {
        // ── flat zone ──
        // 콘의 물리적 반지름 내부 → 최대 cost (물체와 충돌 영역)
        cost = cost_max;
      } else {
        // ── decay zone ──
        // 물리적 반지름 바깥부터 거리에 따라 1/r² 형태로 감쇠
        // d_eff: flat zone 경계로부터의 거리 (inner_radius를 빼줌)
        double d_eff = d - inner_radius;
        cost = cost_max / (1.0 + alpha * d_eff * d_eff);
        // threshold 미만이면 무시 (연산 절약 + 잡음 방지)
        if (cost < threshold) continue;
      }

      // MAX override: 여러 source가 겹칠 때, 가장 강한 자력이 남음
      // 물리적 의미: 가장 위험한(가까운) 장애물의 영향을 유지
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
      cone, cm.cone_cost_max, cm.alpha, cm.cost_threshold,
      cm.cone_radius);
  }

  // ── 차선 적용 ──
  // inner_radius = 0: 차선은 두께가 없으므로 flat zone 없음
  // 중심(차선 위)부터 바로 감쇠 시작, cost_max = 50 (콘보다 약함)
  for (const auto & lane_pt : lanes) {
    apply_source(
      result.data, result.rows, result.cols,
      result.resolution, result.origin_x, result.origin_y,
      lane_pt, cm.lane_cost_max, cm.alpha, cm.cost_threshold,
      0.0);  // inner_radius = 0 (두께 없음)
  }

  result.valid = true;
  return result;
}

}  // namespace planning_mr_ver
