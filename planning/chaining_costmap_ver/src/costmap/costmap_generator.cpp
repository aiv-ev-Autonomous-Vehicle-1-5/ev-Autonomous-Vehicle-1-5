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
#include <limits>

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
  //
  // ── segment 기반 안쪽 코너 패딩 ──
  // heading 극점(부호 반전) 단위로 구간 분할 후, 각 구간마다 안쪽 사이드를
  // 독립 판정하여 per-point padding 적용. S자 커브에서도 올바르게 동작.
  auto apply_chain = [&](const std::vector<ChainedPoint> & chain,
                         const std::vector<double> & extras) {
    for (size_t i = 0; i < chain.size(); ++i) {
      const auto & pt = chain[i];
      const double extra = extras[i];
      const Point2D src = pt.to_point2d();
      if (pt.is_backbone || pt.type == PointType::BBOX) {
        apply_source(
          result.data, result.rows, result.cols,
          result.resolution, result.origin_x, result.origin_y,
          src, cm.bbox_cost_max, cm.sigma, cm.cost_threshold,
          cm.bbox_radius + extra);
      } else {
        apply_source(
          result.data, result.rows, result.cols,
          result.resolution, result.origin_x, result.origin_y,
          src, cm.lane_cost_max, cm.sigma, cm.cost_threshold,
          cm.lane_radius + extra);
      }
    }
  };

  // ── segment 기반 per-point 안쪽 코너 패딩 계산 ──
  std::vector<double> left_extras(left_chain.size(), 0.0);
  std::vector<double> right_extras(right_chain.size(), 0.0);
  if (cm.inner_corner_padding_max > 0.0) {
    compute_per_point_extras(left_chain, right_chain, params,
                             left_extras, right_extras);
  }

  apply_chain(left_chain, left_extras);     // 좌측 경계 비용 적용
  apply_chain(right_chain, right_extras);   // 우측 경계 비용 적용

  // ── unchained 포인트 처리 ──
  // 체이닝 실패 = 좌/우 경계에 배정되지 못한 점
  // BBOX: 정체 불명 장애물 → bbox_cost_max(100)으로 보수적 처리
  // LANE: 차선 포인트 → lane_cost_max(50)으로 처리
  for (const auto & pt : unchained) {
    const Point2D src = pt.to_point2d();
    if (pt.type == PointType::BBOX) {
      apply_source(
        result.data, result.rows, result.cols,
        result.resolution, result.origin_x, result.origin_y,
        src, cm.bbox_cost_max, cm.sigma, cm.cost_threshold,
        cm.bbox_radius);
    } else {
      apply_source(
        result.data, result.rows, result.cols,
        result.resolution, result.origin_x, result.origin_y,
        src, cm.lane_cost_max, cm.sigma, cm.cost_threshold,
        cm.lane_radius);
    }
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
// 중앙선 유인 비용 — backbone 기반 중앙선에 음의 가우시안 적용
// ============================================================================
// 양쪽 backbone이 모두 존재하면 left/right midpoint를 연결한 중앙선을 사용.
// 한쪽 backbone만 존재하면 각 backbone 점의 접선 방향에 수직으로
// track_half_width만큼 오프셋하여 추정 중앙선을 생성한다.
// 양쪽 모두 비어 있을 때만 빈 벡터를 반환한다.
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
  if (left_bb.empty() && right_bb.empty()) return {};

  // 2) raw_center 계산 — 양쪽 존재 시 midpoint, 한쪽만 존재 시 수직 오프셋
  std::vector<Point2D> raw_center;

  if (!left_bb.empty() && !right_bb.empty()) {
    // 양쪽 backbone 존재 → 기존 midpoint 방식
    raw_center.reserve(left_bb.size());
    for (const auto & lp : left_bb) {
      double best_d2 = std::numeric_limits<double>::max();
      int best_j = 0;
      for (int j = 0; j < static_cast<int>(right_bb.size()); ++j) {
        double dx = right_bb[j].x - lp.x;
        double dy = right_bb[j].y - lp.y;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; best_j = j; }
      }
      raw_center.push_back({
        (lp.x + right_bb[best_j].x) * 0.5,
        (lp.y + right_bb[best_j].y) * 0.5
      });
    }
  } else {
    // 한쪽 backbone만 존재 → track_half_width 수직 오프셋으로 centerline 계산
    const auto & bb = left_bb.empty() ? right_bb : left_bb;
    // left only → 시계 방향 90° 회전 (트랙 안쪽 = 우측)
    // right only → 반시계 방향 90° 회전 (트랙 안쪽 = 좌측)
    const double sign = left_bb.empty() ? -1.0 : 1.0;
    const double offset = cm.track_half_width;

    raw_center.reserve(bb.size());
    for (size_t i = 0; i < bb.size(); ++i) {
      // 접선 계산: 양 끝점은 편측 차분, 내부는 중앙 차분
      double tx, ty;
      if (i == 0) {
        tx = bb[1].x - bb[0].x;
        ty = bb[1].y - bb[0].y;
      } else if (i == bb.size() - 1) {
        tx = bb[i].x - bb[i - 1].x;
        ty = bb[i].y - bb[i - 1].y;
      } else {
        tx = bb[i + 1].x - bb[i - 1].x;
        ty = bb[i + 1].y - bb[i - 1].y;
      }
      double len = std::sqrt(tx * tx + ty * ty);
      if (len < 1e-9) continue;
      tx /= len;
      ty /= len;

      // 법선: sign>0(left) → (ty, -tx) 시계90°, sign<0(right) → (-ty, tx) 반시계90°
      double nx = sign * ty;
      double ny = sign * (-tx);

      raw_center.push_back({
        bb[i].x + offset * nx,
        bb[i].y + offset * ny
      });
    }
  }

  // 2.5) midpoint를 resolution 간격으로 리샘플링하여 빈틈 없는 중앙선 생성
  std::vector<Point2D> center_line;
  if (!raw_center.empty()) {
    center_line.push_back(raw_center.front());
    for (size_t i = 1; i < raw_center.size(); ++i) {
      double dx = raw_center[i].x - raw_center[i - 1].x;
      double dy = raw_center[i].y - raw_center[i - 1].y;
      double seg_len = std::sqrt(dx * dx + dy * dy);
      int n = static_cast<int>(std::ceil(seg_len / cm.resolution));
      if (n < 1) n = 1;
      for (int k = 1; k <= n; ++k) {
        double t = static_cast<double>(k) / n;
        center_line.push_back({
          raw_center[i - 1].x + t * dx,
          raw_center[i - 1].y + t * dy
        });
      }
    }
  }

  // 3) 중앙선 각 점에서 음의 가우시안으로 비용 감소
  const double inv_2sigma2 = -1.0 / (2.0 * cm.center_attract_sigma * cm.center_attract_sigma);
  const double r_total = cm.center_attract_sigma * 3.0;  // 3σ까지만 순회
  const int r_cells = static_cast<int>(std::ceil(r_total / cm.resolution));

  int cpt_idx = 0;
  for (const auto & cpt : center_line) {
    int src_col = static_cast<int>(std::round((cpt.x - costmap.origin_x) / cm.resolution));
    int src_row = static_cast<int>(std::round((cpt.y - costmap.origin_y) / cm.resolution));

    // 중앙선 포인트 자체 셀의 before 값 로그 (처음 5개만)
    if (cpt_idx < 5) {
      int center_idx = src_row * costmap.cols + src_col;
      if (center_idx >= 0 && center_idx < static_cast<int>(costmap.data.size())) {
        double before = costmap.data[center_idx];
        bool skipped = (before >= cm.bbox_cost_max);
        std::fprintf(stderr,
          "[center_attract] pt[%d] (%.2f,%.2f) grid(%d,%d) before=%.1f bbox_max=%.1f skipped=%d\n",
          cpt_idx, cpt.x, cpt.y, src_row, src_col, before, cm.bbox_cost_max, skipped);
      }
    }

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
        double after = std::max(0.0, costmap.data[idx] - reduction);

        // 처음 5개 포인트의 중심 셀만 after 로그
        if (cpt_idx < 5 && r == src_row && c == src_col) {
          std::fprintf(stderr,
            "[center_attract] pt[%d] reduction=%.1f after=%.1f\n",
            cpt_idx, reduction, after);
        }

        costmap.data[idx] = after;
      }
    }
    ++cpt_idx;
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

// ============================================================================
// compute_turning_signs — chain 각 점의 회전 부호 계산
// ============================================================================
//
// [동작]
//   연속 3점(i-1, i, i+1)의 외적 부호로 좌/우회전 판정.
//   양수(+1) = 좌회전(반시계), 음수(-1) = 우회전(시계).
//   |외적| < eps 이면 직진(0).
//   첫/끝 점은 인접 점의 부호를 상속.
//
// ============================================================================

std::vector<int> CostmapGenerator::compute_turning_signs(
  const std::vector<ChainedPoint> & chain)
{
  const size_t n = chain.size();
  std::vector<int> signs(n, 0);
  if (n < 3) return signs;

  constexpr double eps = 1e-6;
  for (size_t i = 1; i + 1 < n; ++i) {
    // 외적: (P[i]-P[i-1]) × (P[i+1]-P[i])
    double ax = chain[i].x - chain[i - 1].x;
    double ay = chain[i].y - chain[i - 1].y;
    double bx = chain[i + 1].x - chain[i].x;
    double by = chain[i + 1].y - chain[i].y;
    double cross = ax * by - ay * bx;

    if (cross > eps)       signs[i] = +1;  // 좌회전
    else if (cross < -eps) signs[i] = -1;  // 우회전
    // else 0 (직진)
  }

  // 첫/끝 점은 인접 점의 부호 상속
  signs[0]     = signs[1];
  signs[n - 1] = signs[n - 2];
  return signs;
}

// ============================================================================
// split_segments — 부호 배열에서 극점 단위 구간 분할
// ============================================================================
//
// [동작]
//   1) 부호가 바뀌는 지점에서 구간 분할
//   2) min_segment_len 미만 구간은 인접 구간에 병합 (노이즈 방지)
//   3) 부호 0(직진) 구간은 독립 구간으로 유지
//
// ============================================================================

std::vector<CostmapGenerator::Segment> CostmapGenerator::split_segments(
  const std::vector<int> & signs,
  size_t min_segment_len)
{
  std::vector<Segment> segs;
  if (signs.empty()) return segs;

  // 1) 부호 변화 지점에서 분할
  size_t begin = 0;
  int cur_sign = signs[0];
  for (size_t i = 1; i < signs.size(); ++i) {
    if (signs[i] != cur_sign) {
      segs.push_back({begin, i, cur_sign});
      begin = i;
      cur_sign = signs[i];
    }
  }
  segs.push_back({begin, signs.size(), cur_sign});

  // 2) 짧은 구간 병합 — 인접 구간에 흡수
  bool merged = true;
  while (merged) {
    merged = false;
    for (size_t i = 0; i < segs.size(); ++i) {
      size_t len = segs[i].end - segs[i].begin;
      if (len >= min_segment_len) continue;

      // 짧은 구간 → 인접 중 더 긴 쪽에 병합
      if (i > 0 && (i + 1 >= segs.size() ||
          (segs[i - 1].end - segs[i - 1].begin) >=
          (segs[i + 1].end - segs[i + 1].begin))) {
        // 이전 구간에 병합
        segs[i - 1].end = segs[i].end;
        segs.erase(segs.begin() + static_cast<long>(i));
      } else if (i + 1 < segs.size()) {
        // 다음 구간에 병합
        segs[i + 1].begin = segs[i].begin;
        segs.erase(segs.begin() + static_cast<long>(i));
      }
      merged = true;
      break;  // 처음부터 다시
    }
  }

  return segs;
}

// ============================================================================
// compute_segment_curvature — 구간 [begin, end) 내 최대 |곡률| 계산
// ============================================================================
//
// [동작]
//   Menger 곡률: kappa(i) = 2 * |cross(a, b)| / (|a| * |b| * |c|)
//   범위를 [begin, end)로 제한하여 해당 구간의 최대 곡률만 반환.
//
// ============================================================================

double CostmapGenerator::compute_segment_curvature(
  const std::vector<ChainedPoint> & chain,
  size_t begin, size_t end)
{
  if (end - begin < 3) return 0.0;

  double max_kappa = 0.0;
  for (size_t i = begin + 1; i + 1 < end; ++i) {
    double ax = chain[i].x - chain[i - 1].x;
    double ay = chain[i].y - chain[i - 1].y;
    double bx = chain[i + 1].x - chain[i].x;
    double by = chain[i + 1].y - chain[i].y;
    double la = std::sqrt(ax * ax + ay * ay);
    double lb = std::sqrt(bx * bx + by * by);
    double cx = chain[i + 1].x - chain[i - 1].x;
    double cy = chain[i + 1].y - chain[i - 1].y;
    double lc = std::sqrt(cx * cx + cy * cy);
    double denom = la * lb * lc;
    if (denom < 1e-9) continue;

    double cross = ax * by - ay * bx;
    double kappa = std::abs(2.0 * cross / denom);
    if (kappa > max_kappa) max_kappa = kappa;
  }
  return max_kappa;
}

// ============================================================================
// compute_per_point_extras — segment 기반 per-point 안쪽 코너 패딩
// ============================================================================
//
// [동작]
//   1) ref chain(더 긴 쪽)의 turning signs 계산
//   2) 극점 단위로 segment 분할
//   3) 각 segment마다:
//      - sign으로 안쪽 사이드 판정 (+1=좌회전→left 안쪽)
//      - ref chain 인덱스를 inner chain 인덱스로 비례 매핑
//      - inner chain 해당 범위의 max curvature 계산
//      - threshold~차량한계 범위로 정규화하여 padding_min~max 보간
//   4) 경계 테이퍼링: 구간 경계에서 패딩 급변 방지
//
// ============================================================================

void CostmapGenerator::compute_per_point_extras(
  const std::vector<ChainedPoint> & left_chain,
  const std::vector<ChainedPoint> & right_chain,
  const PlanningParams & params,
  std::vector<double> & left_extras,
  std::vector<double> & right_extras)
{
  const auto & cm = params.costmap;
  if (left_chain.empty() || right_chain.empty()) return;

  // ref chain = 더 긴 쪽 (회전 방향 판정 기준)
  bool ref_is_left = (left_chain.size() >= right_chain.size());
  const auto & ref_chain   = ref_is_left ? left_chain : right_chain;
  const auto & other_chain = ref_is_left ? right_chain : left_chain;

  // 1) ref chain의 turning signs + segment 분할
  auto signs = compute_turning_signs(ref_chain);
  auto segs  = split_segments(signs);

  // 차량 최대 곡률: κ_max = tan(δ_max) / wheelbase
  const double kappa_max = 1.0 / params.vehicle.r_min();
  const double kappa_range = kappa_max - cm.corner_curvature_threshold;
  if (kappa_range <= 0.0) return;  // threshold가 차량 한계 이상이면 패딩 불가

  // 2) 각 segment 처리
  for (const auto & seg : segs) {
    if (seg.sign == 0) continue;  // 직진 구간은 패딩 없음

    // 안쪽 사이드 판정:
    //   ref가 left일 때: sign>0(좌회전) → left가 안쪽
    //   ref가 right일 때: sign>0 → ref(right)에서 좌회전 = right가 안쪽
    bool inner_is_ref = (seg.sign > 0);
    // ref가 left이고 inner_is_ref → left 안쪽
    // ref가 left이고 !inner_is_ref → right(other) 안쪽
    bool inner_is_left = ref_is_left ? inner_is_ref : !inner_is_ref;

    const auto & inner_chain = inner_is_left ? left_chain : right_chain;
    auto & inner_extras      = inner_is_left ? left_extras : right_extras;

    // ref chain 구간 인덱스를 inner chain 인덱스로 비례 매핑
    size_t inner_begin, inner_end;
    if (&inner_chain == &ref_chain) {
      // inner = ref → 인덱스 그대로
      inner_begin = seg.begin;
      inner_end   = seg.end;
    } else {
      // inner = other → 비례 매핑
      double scale = (other_chain.size() <= 1) ? 0.0
                   : static_cast<double>(other_chain.size() - 1)
                   / static_cast<double>(ref_chain.size() - 1);
      inner_begin = static_cast<size_t>(seg.begin * scale);
      inner_end   = static_cast<size_t>(
        std::min(static_cast<size_t>(std::ceil(seg.end * scale)),
                 other_chain.size()));
    }
    if (inner_end <= inner_begin) continue;

    // 구간 내 최대 곡률
    double seg_kappa = compute_segment_curvature(inner_chain, inner_begin, inner_end);
    if (seg_kappa < cm.corner_curvature_threshold) continue;

    // threshold ~ 차량한계 범위로 정규화
    double ratio = (seg_kappa - cm.corner_curvature_threshold) / kappa_range;
    ratio = std::clamp(ratio, 0.0, 1.0);
    double padding = cm.inner_corner_padding_min
                   + ratio * (cm.inner_corner_padding_max - cm.inner_corner_padding_min);

    // inner chain 해당 구간에 패딩 할당
    for (size_t i = inner_begin; i < inner_end && i < inner_extras.size(); ++i) {
      inner_extras[i] = std::max(inner_extras[i], padding);
    }
  }

  // 3) 경계 테이퍼링 — 패딩 급변 방지 (5점 선형 ramp)
  auto taper = [](std::vector<double> & extras) {
    constexpr size_t TAPER = 5;
    const size_t n = extras.size();
    if (n < 2) return;
    for (size_t i = 1; i < n; ++i) {
      double diff = std::abs(extras[i] - extras[i - 1]);
      if (diff < 1e-6) continue;
      // 패딩이 0→X 또는 X→0 으로 변하는 경계 감지
      size_t ramp_len = std::min(TAPER, std::min(i, n - i));
      if (extras[i - 1] < extras[i]) {
        // 0→X: i 이전 ramp_len개 점에 선형 증가
        for (size_t k = 0; k < ramp_len; ++k) {
          size_t idx = i - 1 - k;
          double t = static_cast<double>(k + 1) / (ramp_len + 1);
          double ramped = extras[i] * (1.0 - t);
          extras[idx] = std::max(extras[idx], ramped);
        }
      } else {
        // X→0: i 이후 ramp_len개 점에 선형 감소
        for (size_t k = 0; k < ramp_len && (i + k) < n; ++k) {
          double t = static_cast<double>(k + 1) / (ramp_len + 1);
          double ramped = extras[i - 1] * (1.0 - t);
          extras[i + k] = std::max(extras[i + k], ramped);
        }
      }
    }
  };
  taper(left_extras);
  taper(right_extras);
}

}  // namespace chaining_costmap_ver
