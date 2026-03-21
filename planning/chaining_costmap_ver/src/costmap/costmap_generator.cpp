/**
 * @file costmap_generator.cpp
 * @brief 가우시안 비용 지도(Costmap) 생성기 — 구현부
 *
 * 좌/우 경계 체인의 각 점(ChainedPoint)에서 가우시안 비용장을 방사하여
 * 2D 격자 비용 지도를 생성한다.
 *
 * ── 비용 분기 ──
 *   backbone  : is_backbone=true → 타입 무관하게 bbox_cost_max + bbox_radius 적용
 *               (lane↔bbox 전환 구간에서도 끊김 없는 강한 비용 장벽 형성)
 *   BBOX      : bbox_cost_max(100) + bbox_radius flat zone + 가우시안 감쇠
 *   LANE      : lane_cost_max(50)  + lane_radius flat zone + 가우시안 감쇠
 *   unchained : bbox 비용으로 보수적 처리 (체이닝 실패 = 미확인 장애물)
 *
 * ── max-merge 정책 ──
 *   여러 source의 비용이 같은 셀에 겹치면, 큰 값을 유지한다.
 *   (덧셈이 아닌 max 연산 → bbox가 밀집해도 비용이 무한히 커지지 않음)
 */
#include "chaining_costmap_ver/costmap/costmap_generator.hpp"

#include <cmath>
#include <algorithm>

namespace chaining_costmap_ver
{

// ============================================================================
// effective_radius — 가우시안 유효 반경 계산
// ============================================================================
//
// [목적]
//   apply_source()에서 순회할 셀 범위를 제한하기 위한 반경을 계산한다.
//   이 반경 바깥에서는 가우시안 비용이 threshold 미만이므로 계산할 필요 없다.
//
// [수식 유도]
//   cost(d) = cost_max * exp(-d² / (2σ²)) = threshold 라 하면,
//   exp(-d² / (2σ²)) = threshold / cost_max
//   -d² / (2σ²) = ln(threshold / cost_max) = -ln(cost_max / threshold)
//   d² = 2σ² * ln(cost_max / threshold)
//   d = σ * √(2 * ln(cost_max / threshold))
//
// [예시]
//   cost_max=100, sigma=1.0, threshold=2.0
//   → ratio = 100/2 = 50
//   → d = 1.0 * √(2 * ln(50)) = 1.0 * √(2 * 3.912) ≈ 2.80m
//   → 2.80m 바깥에서는 비용이 2.0 미만 → 무시해도 됨
//
// [경계 처리]
//   threshold ≤ 0 또는 sigma ≤ 0 → 안전하게 100m 반환 (전체 격자 순회)
//   cost_max ≤ threshold → 반경 0 반환 (어디서나 threshold 미만)
//
// ============================================================================

double CostmapGenerator::effective_radius(
  double cost_max, double sigma, double threshold)
{
  if (threshold <= 0.0 || sigma <= 0.0) return 100.0;  // 안전 폴백
  double ratio = cost_max / threshold;
  if (ratio <= 1.0) return 0.0;   // cost_max가 threshold 이하 → 유효 반경 없음
  return sigma * std::sqrt(2.0 * std::log(ratio));
}

// ============================================================================
// apply_source — 단일 source의 가우시안 비용을 격자에 적용
// ============================================================================
//
// [동작]
//   1) effective_radius + inner_radius = 총 유효 반경 r_total 계산
//   2) r_total을 셀 단위로 변환 → source 주변 (2*r_cells+1)² 영역만 순회
//   3) 각 셀에 대해:
//      - 셀 중심 ↔ source 거리 d 계산
//      - d ≤ inner_radius → cost = cost_max (flat zone)
//      - d > inner_radius → cost = cost_max * exp(-(d-inner_radius)² / (2σ²))
//      - cost < threshold → 건너뜀 (미미한 비용)
//      - cost > grid[셀] → 갱신 (max-merge)
//
// [성능 최적화]
//   - inv_2sigma2를 미리 계산하여 루프 내 나눗셈 제거
//   - effective_radius로 순회 범위 제한 → O(πr²/res²) 셀만 처리
//     전체 격자 O(rows×cols) 대비 대폭 절감
//
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
  // 유효 반경: inner_radius(flat zone) + 가우시안 감쇠 거리
  double r_decay = effective_radius(cost_max, sigma, threshold);
  double r_total = inner_radius + r_decay;
  int r_cells = static_cast<int>(std::ceil(r_total / resolution));

  // 가우시안 지수부의 상수: -1 / (2σ²)
  // exp(inv_2sigma2 * d²) = exp(-d² / (2σ²))
  // 루프 내에서 나눗셈 대신 곱셈으로 처리하여 성능 향상
  const double inv_2sigma2 = -1.0 / (2.0 * sigma * sigma);

  // source의 그리드 좌표 (가장 가까운 셀)
  int src_col = static_cast<int>(std::round((source.x - origin_x) / resolution));
  int src_row = static_cast<int>(std::round((source.y - origin_y) / resolution));

  // 순회 범위를 격자 경계 내로 클리핑
  int row_min = std::max(0, src_row - r_cells);
  int row_max = std::min(rows - 1, src_row + r_cells);
  int col_min = std::max(0, src_col - r_cells);
  int col_max = std::min(cols - 1, src_col + r_cells);

  for (int r = row_min; r <= row_max; ++r) {
    for (int c = col_min; c <= col_max; ++c) {
      // 셀 중심의 월드 좌표 (+0.5: 셀 중심)
      double wx = origin_x + (c + 0.5) * resolution;
      double wy = origin_y + (r + 0.5) * resolution;

      // source ↔ 셀 중심 거리
      double dx = wx - source.x;
      double dy = wy - source.y;
      double d = std::sqrt(dx * dx + dy * dy);

      double cost;
      if (d <= inner_radius) {
        // ── flat zone ──
        // bbox/lane의 flat zone(inner_radius) 이내 → 최대 비용 유지
        cost = cost_max;
      } else {
        // ── 가우시안 감쇠 ──
        // d_eff: flat zone 바깥 경계로부터의 거리
        // cost = cost_max * exp(-d_eff² / (2σ²))
        double d_eff = d - inner_radius;
        cost = cost_max * std::exp(inv_2sigma2 * d_eff * d_eff);
        if (cost < threshold) continue;  // 미미한 비용은 건너뜀
      }

      // ── max-merge ──
      // 여러 source가 겹치면 큰 값을 유지한다.
      // 덧셈이 아닌 max: 밀집된 bbox가 비정상적으로 높은 비용을 만들지 않도록.
      int idx = r * cols + c;
      if (cost > grid[idx]) {
        grid[idx] = cost;
      }
    }
  }
}

// ============================================================================
// generate — costmap 생성 메인 함수
// ============================================================================
//
// [격자 좌표계]
//   origin = (origin_x, -size_y/2)  — origin_x는 파라미터로 설정
//   → X축: origin_x ~ origin_x + size_x (origin_x < 0이면 후방도 포함)
//   → Y축: -size_y/2 ~ +size_y/2 (좌우 대칭)
//   → col = X방향, row = Y방향
//
// [처리 순서]
//   1) 격자 메타정보(rows, cols, origin 등) 설정
//   2) 전체 격자를 0.0(자유 공간)으로 초기화
//   3) 좌측 체인의 각 점에 가우시안 비용 적용 (BBOX/LANE 분기)
//   4) 우측 체인의 각 점에 가우시안 비용 적용
//   5) unchained 포인트에 BBOX 비용 적용 (보수적 처리)
//
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
  result.cols = static_cast<int>(std::round(cm.size_x / cm.resolution));  // X방향 셀 수
  result.rows = static_cast<int>(std::round(cm.size_y / cm.resolution));  // Y방향 셀 수
  result.origin_x = cm.origin_x;         // X 원점: 파라미터로 설정 (base_link 기준)
  result.origin_y = -cm.size_y / 2.0;   // Y 원점: 좌우 대칭을 위해 -size_y/2

  // 전체 격자를 0.0(자유 공간)으로 초기화
  result.data.assign(result.rows * result.cols, 0.0);

  // ── 체인 포인트 비용 적용 (backbone / BBOX / LANE 분기) ──
  // backbone:  is_backbone=true → bbox_cost_max + bbox_radius (타입 무관, 단단한 벽)
  // BBOX:      bbox_cost_max + bbox_radius flat zone → 강한 비용 장벽
  // LANE:      lane_cost_max + lane_radius flat zone → 약한 비용 장벽 (넘을 수 있음)
  auto apply_chain = [&](const std::vector<ChainedPoint> & chain) {
    for (const auto & pt : chain) {
      const Point2D src = pt.to_point2d();
      if (pt.is_backbone || pt.type == PointType::BBOX) {
        // backbone 포인트는 타입에 관계없이 bbox_cost_max 적용
        // → lane↔bbox 전환 구간에서도 끊김 없는 비용 장벽 형성
        apply_source(
          result.data, result.rows, result.cols,
          result.resolution, result.origin_x, result.origin_y,
          src, cm.bbox_cost_max, cm.sigma, cm.cost_threshold,
          cm.bbox_radius);   // flat zone = bbox_radius
      } else {
        apply_source(
          result.data, result.rows, result.cols,
          result.resolution, result.origin_x, result.origin_y,
          src, cm.lane_cost_max, cm.sigma, cm.cost_threshold,
          cm.lane_radius);   // flat zone = lane_radius
      }
    }
  };

  apply_chain(left_chain);    // 좌측 경계 비용 적용
  apply_chain(right_chain);   // 우측 경계 비용 적용

  // ── unchained 포인트 처리 ──
  // 체이닝 실패 = 좌/우 경계에 배정되지 못한 점
  // 정체를 알 수 없으므로 bbox 비용(가장 높은 비용)으로 보수적 처리
  // → A*가 이 점들을 최대한 회피하도록 유도
  for (const auto & pt : unchained) {
    const Point2D src = pt.to_point2d();
    apply_source(
      result.data, result.rows, result.cols,
      result.resolution, result.origin_x, result.origin_y,
      src, cm.bbox_cost_max, cm.sigma, cm.cost_threshold,
      cm.bbox_radius);
  }

  result.valid = true;
  return result;
}

// ============================================================================
// apply_entry_walls — costmap 하단→시드까지 가상 bbox 벽 생성
// ============================================================================
//
// [목적]
//   A*가 경계 뒤쪽(바깥)으로 돌아가는 경로를 생성하는 것을 방지한다.
//   costmap 하단(origin_x) 양옆에서 시드까지 가상 bbox를 일정 간격으로 배치하여
//   "입구(시드 사이)"로만 진입하도록 유도한다.

// ============================================================================
// 중앙선 유인 비용 — 양쪽 backbone 중점에 음의 가우시안 적용
// ============================================================================
// left/right backbone의 중점을 연결한 중앙선에 비용 감소를 적용하여
// A*가 자연스럽게 중앙으로 유도되게 한다.
// cost >= bbox_cost_max인 장애물 셀은 건드리지 않는다.
// ============================================================================

std::vector<Point2D> CostmapGenerator::apply_center_attraction(
  CostmapResult & costmap,
  const std::vector<ChainedPoint> & left_chain,
  const std::vector<ChainedPoint> & right_chain,
  const PlanningParams & params)
{
  const auto & cm = params.costmap;
  if (cm.center_attract_max <= 0.0 || cm.center_attract_sigma <= 0.0) return {};

  // 1) left/right에서 is_backbone 점만 추출
  std::vector<Point2D> left_bb, right_bb;
  for (const auto & pt : left_chain) {
    if (pt.is_backbone) left_bb.push_back({pt.x, pt.y});
  }
  for (const auto & pt : right_chain) {
    if (pt.is_backbone) right_bb.push_back({pt.x, pt.y});
  }
  if (left_bb.empty() || right_bb.empty()) return {};

  // 2) left 각 점에 대해 right 최근접 매칭 → midpoint 계산
  std::vector<Point2D> center_line;
  center_line.reserve(left_bb.size());
  for (const auto & lp : left_bb) {
    double best_d2 = std::numeric_limits<double>::max();
    int best_j = 0;
    for (int j = 0; j < static_cast<int>(right_bb.size()); ++j) {
      double dx = right_bb[j].x - lp.x;
      double dy = right_bb[j].y - lp.y;
      double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; best_j = j; }
    }
    center_line.push_back({
      (lp.x + right_bb[best_j].x) * 0.5,
      (lp.y + right_bb[best_j].y) * 0.5
    });
  }

  // 3) 중앙선 각 점에서 음의 가우시안으로 비용 감소
  const double inv_2sigma2 = -1.0 / (2.0 * cm.center_attract_sigma * cm.center_attract_sigma);
  const double r_total = cm.center_attract_sigma * 3.0;  // 3σ까지만 순회
  const int r_cells = static_cast<int>(std::ceil(r_total / cm.resolution));

  for (const auto & cpt : center_line) {
    int src_col = static_cast<int>(std::round((cpt.x - costmap.origin_x) / cm.resolution));
    int src_row = static_cast<int>(std::round((cpt.y - costmap.origin_y) / cm.resolution));

    int row_min = std::max(0, src_row - r_cells);
    int row_max = std::min(costmap.rows - 1, src_row + r_cells);
    int col_min = std::max(0, src_col - r_cells);
    int col_max = std::min(costmap.cols - 1, src_col + r_cells);

    for (int r = row_min; r <= row_max; ++r) {
      for (int c = col_min; c <= col_max; ++c) {
        int idx = r * costmap.cols + c;

        // 장애물 보존: bbox_cost_max 이상이면 skip
        if (costmap.data[idx] >= cm.bbox_cost_max) continue;

        double wx = costmap.origin_x + (c + 0.5) * cm.resolution;
        double wy = costmap.origin_y + (r + 0.5) * cm.resolution;
        double ddx = wx - cpt.x;
        double ddy = wy - cpt.y;
        double d2 = ddx * ddx + ddy * ddy;

        double reduction = cm.center_attract_max * std::exp(inv_2sigma2 * d2);
        costmap.data[idx] = std::max(0.0, costmap.data[idx] - reduction);
      }
    }
  }

  return center_line;
}

//
// [가상 bbox 배치]
//   좌측 벽: (origin_x, +entry_wall_ego_y) → left_seed
//   우측 벽: (origin_x, -entry_wall_ego_y) → right_seed
//   from→to 직선을 step(=resolution*2) 간격으로 분할하여 가상 bbox 배치.
//   → costmap 하단부터 시드까지 연속적인 비용 장벽 형성.
//   → 차량이 costmap 중앙에 있어도 뒤쪽이 완전히 막혀 A*가 우회 불가.
//
// [왜 resolution*2 간격인가?]
//   sigma=1.0m일 때 가우시안의 3σ≈3m이므로, resolution*2(≈0.3m) 간격이면
//   인접 bbox의 비용장이 충분히 겹쳐서 틈이 없는 연속 벽이 만들어진다.
//
// ============================================================================

void CostmapGenerator::apply_entry_walls(
  CostmapResult & costmap,
  const Point2D & left_seed,
  const Point2D & right_seed,
  const PlanningParams & params)
{
  const auto & cm = params.costmap;
  const double step = cm.resolution * 2.0;  // 가상 bbox 간격 (resolution의 2배)

  // costmap 하단 x좌표 (= origin_x, 그리드 좌하단의 x값)
  const double bottom_x = costmap.origin_x;

  // from→to 직선을 따라 가상 bbox를 등간격 샘플링하는 람다
  auto sample_bboxes = [&](const Point2D & from, const Point2D & to) {
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double len = std::sqrt(dx * dx + dy * dy);
    if (len < step) return;  // 너무 짧으면 벽 불필요

    int n = static_cast<int>(std::ceil(len / step));  // 분할 수
    for (int i = 0; i <= n; ++i) {
      double t = static_cast<double>(i) / n;  // 보간 비율 [0, 1]
      Point2D pt = {from.x + t * dx, from.y + t * dy};  // 선형 보간
      apply_source(
        costmap.data, costmap.rows, costmap.cols,
        costmap.resolution, costmap.origin_x, costmap.origin_y,
        pt, cm.bbox_cost_max, cm.sigma, cm.cost_threshold, cm.bbox_radius);
    }
  };

  // 좌측 벽: costmap 하단 좌측(origin_x, +entry_wall_ego_y) → left_seed
  sample_bboxes({bottom_x, +cm.entry_wall_ego_y}, left_seed);
  // 우측 벽: costmap 하단 우측(origin_x, -entry_wall_ego_y) → right_seed
  sample_bboxes({bottom_x, -cm.entry_wall_ego_y}, right_seed);
}

}  // namespace chaining_costmap_ver
