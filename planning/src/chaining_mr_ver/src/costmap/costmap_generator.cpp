/**
 * @file costmap_generator.cpp
 * @brief Magnetic Resistance Costmap 생성기 — 구현부 (체인 입력 버전)
 *
 * ──────────────────────────────────────────────────────────────────
 * [이 파일이 하는 일]
 *
 * 좌/우 경계 체인의 각 점(ChainedPoint)을 "자석"으로 취급하여,
 * 2D 격자(grid) 위에 가우시안 모양의 비용장(cost field)을 뿌린다.
 * 결과물인 costmap은 MagneticPlanner에게 전달되어,
 * 경계에서 최대한 멀리 떨어진(= 비용이 낮은) 경로를 탐색하게 된다.
 *
 * [CONE vs LANE 차별 적용]
 *
 *   - CONE (PE 드럼/교통 콘):
 *     - cone_cost_max (예: 100) → 높은 반발력
 *     - cone_radius (예: 0.65m) → flat zone 있음
 *       → 콘의 물리적 크기 + 안전 마진을 고려한 절대 금지 구역
 *     - flat zone 밖부터 가우시안 감쇠 시작
 *
 *   - LANE (카메라 차선 인식점):
 *     - lane_cost_max (예: 50) → 콘보다 약한 반발력
 *     - inner_radius = 0 → flat zone 없음
 *       → 차선은 물리적 크기가 없으므로, 중심에서 바로 감쇠
 *     - 차선을 약간 밟는 것은 콘을 치는 것보다 덜 위험하다는 의미
 *
 * ──────────────────────────────────────────────────────────────────
 */
#include "chaining_mr_ver/costmap/costmap_generator.hpp"

#include <cmath>
#include <algorithm>

namespace chaining_mr_ver
{

// ============================================================================
// effective_radius — 가우시안 비용이 threshold 이상인 최대 거리 계산
// ============================================================================

/**
 * @brief 가우시안 역함수를 이용해, 비용이 threshold 이상인 최대 반경을 계산
 *
 * [수학적 유도]
 *
 *   cost(r) = cost_max * exp(-r² / (2σ²))
 *
 *   cost(r) >= threshold 를 만족하는 최대 r을 구하면:
 *
 *     cost_max * exp(-r² / (2σ²)) = threshold
 *     exp(-r² / (2σ²)) = threshold / cost_max
 *     -r² / (2σ²) = ln(threshold / cost_max)
 *     r² = -2σ² * ln(threshold / cost_max)
 *        = 2σ² * ln(cost_max / threshold)
 *     r  = σ * sqrt(2 * ln(cost_max / threshold))
 *
 * [왜 필요한가?]
 *
 *   그리드의 모든 셀에 대해 비용을 계산하면 O(rows * cols * num_points)로
 *   매우 느리다. effective_radius를 구하면, 각 source 주변의 작은 사각형
 *   영역만 순회하면 되므로 계산량이 크게 줄어든다.
 *
 *   예) cost_max=100, sigma=1.0, threshold=2.0
 *       → r = 1.0 * sqrt(2 * ln(50)) ≈ 2.80m
 *       → 2.80m 밖의 셀은 비용이 2.0 미만이므로 무시 가능
 *
 * @param cost_max   경계점 중심의 최대 비용
 * @param sigma      가우시안 표준편차 [m]
 * @param threshold  최소 유효 비용 (이 이하는 무시)
 * @return 유효 반경 [m]. 이 거리 밖의 셀은 threshold 미만.
 */
double CostmapGenerator::effective_radius(
  double cost_max, double sigma, double threshold)
{
  // 예외 처리: threshold나 sigma가 0 이하이면 안전하게 큰 값 반환
  if (threshold <= 0.0 || sigma <= 0.0) return 100.0;

  // ratio = cost_max / threshold (예: 100 / 2 = 50)
  double ratio = cost_max / threshold;

  // ratio <= 1이면 cost_max <= threshold → 비용이 threshold를 넘지 못함 → 반경 0
  if (ratio <= 1.0) return 0.0;

  // r = σ * sqrt(2 * ln(ratio))
  return sigma * std::sqrt(2.0 * std::log(ratio));
}

// ============================================================================
// apply_source — 단일 경계점의 가우시안 비용장을 그리드에 적용
// ============================================================================

/**
 * @brief 단일 경계점(source)이 주변 셀에 "밀어내는 비용"을 뿌리는 핵심 함수
 *
 * [알고리즘 단계별 설명]
 *
 *   Step 1: 유효 반경 계산
 *     - r_decay = effective_radius(cost_max, sigma, threshold)
 *       → 가우시안 감쇠 구간에서 threshold 이상인 최대 거리
 *     - r_total = inner_radius + r_decay
 *       → flat zone + 감쇠 구간을 합친 전체 유효 반경
 *     - r_cells = ceil(r_total / resolution)
 *       → 셀 단위로 변환 (순회할 사각형 범위의 반변)
 *
 *   Step 2: 사전 계산
 *     - inv_2sigma2 = -1 / (2σ²)
 *       → 가우시안 지수부의 상수를 미리 계산 (반복 나눗셈 방지)
 *
 *   Step 3: source의 그리드 좌표 계산
 *     - src_col = round((source.x - origin_x) / resolution)
 *     - src_row = round((source.y - origin_y) / resolution)
 *       → 월드 좌표 → 그리드 인덱스 변환
 *
 *   Step 4: 순회 범위 결정 (클리핑)
 *     - [row_min, row_max] × [col_min, col_max]
 *     - source 주변 ±r_cells 범위, 그리드 경계로 클리핑
 *       → 전체 그리드를 돌지 않고 source 주변만 처리
 *
 *   Step 5: 각 셀에 대해 비용 계산 및 병합
 *     5a) 셀의 월드 좌표 역산: wx, wy (셀 중심점)
 *     5b) source까지의 유클리드 거리 d 계산
 *     5c) 비용 결정:
 *         - d <= inner_radius → cost = cost_max (flat zone, 진입 금지)
 *         - d >  inner_radius → d_eff = d - inner_radius
 *                                cost = cost_max * exp(inv_2sigma2 * d_eff²)
 *         - cost < threshold → 건너뜀 (의미 없는 미세 비용)
 *     5d) max-merge: grid[idx] = max(grid[idx], cost)
 *         → 여러 source가 겹치는 영역에서 최대값만 유지
 *
 * [비용 프로파일 그림 (CONE, inner_radius > 0)]
 *
 *   cost
 *   cost_max ┤████████████████
 *            │                ████
 *            │                    ████
 *   threshold┤────────────────────────████───── (이 이하 무시)
 *            │                            ████
 *         0  └──────────────────────────────── d
 *            0      inner_r   inner_r+σ    r_total
 *            ←──flat zone──→←──가우시안 감쇠──→
 *
 * [비용 프로파일 그림 (LANE, inner_radius = 0)]
 *
 *   cost
 *   cost_max ┤█
 *            │ ████
 *            │     ████
 *   threshold┤─────────████───── (이 이하 무시)
 *            │             ████
 *         0  └──────────────── d
 *            0    σ    2σ   r_total
 *            ←──가우시안 감쇠 시작──→
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
  // ── Step 1: 유효 반경 계산 ──
  // r_decay: 가우시안 감쇠 구간에서 threshold 이상인 최대 거리
  double r_decay = effective_radius(cost_max, sigma, threshold);
  // r_total: flat zone(inner_radius) + 감쇠 구간(r_decay) = 전체 유효 반경
  double r_total = inner_radius + r_decay;
  // r_cells: 셀 단위로 변환 (올림) → 순회할 사각형의 반변 길이
  int r_cells = static_cast<int>(std::ceil(r_total / resolution));

  // ── Step 2: 가우시안 지수부 상수 사전 계산 ──
  // exp(inv_2sigma2 * d²) = exp(-d² / (2σ²))
  // 루프 안에서 매번 나눗셈하지 않기 위해 미리 계산
  const double inv_2sigma2 = -1.0 / (2.0 * sigma * sigma);

  // ── Step 3: source의 그리드 좌표 계산 ──
  // 월드 좌표 → 그리드 인덱스 (가장 가까운 셀)
  // 예) source.x=2.0, origin_x=-5.0, resolution=0.05
  //     → src_col = round((2.0 - (-5.0)) / 0.05) = round(140) = 140
  int src_col = static_cast<int>(std::round((source.x - origin_x) / resolution));
  int src_row = static_cast<int>(std::round((source.y - origin_y) / resolution));

  // ── Step 4: 순회 범위 결정 (그리드 경계로 클리핑) ──
  // source 주변 ±r_cells 범위만 순회 (전체 그리드 순회 방지)
  int row_min = std::max(0, src_row - r_cells);
  int row_max = std::min(rows - 1, src_row + r_cells);
  int col_min = std::max(0, src_col - r_cells);
  int col_max = std::min(cols - 1, src_col + r_cells);

  // ── Step 5: 각 셀에 대해 비용 계산 및 max-merge ──
  for (int r = row_min; r <= row_max; ++r) {
    for (int c = col_min; c <= col_max; ++c) {

      // Step 5a: 셀 (r, c)의 월드 좌표 역산 (셀 중심점)
      // +0.5를 더하는 이유: 셀의 좌하단이 아닌 중심점을 기준으로 거리 계산
      double wx = origin_x + (c + 0.5) * resolution;
      double wy = origin_y + (r + 0.5) * resolution;

      // Step 5b: source까지의 유클리드 거리 계산
      double dx = wx - source.x;
      double dy = wy - source.y;
      double d = std::sqrt(dx * dx + dy * dy);

      // Step 5c: 거리에 따른 비용 결정
      double cost;
      if (d <= inner_radius) {
        // ★ Flat Zone: inner_radius 이내 → 무조건 최대 비용
        // CONE의 경우: 콘의 물리적 크기 + 안전 마진 안쪽은 절대 진입 금지
        // LANE의 경우: inner_radius=0이므로 이 분기에 들어오지 않음
        cost = cost_max;
      } else {
        // ★ 가우시안 감쇠 구간: flat zone 경계에서부터의 거리로 계산
        // d_eff = d - inner_radius
        //   → CONE: 콘 표면으로부터의 거리
        //   → LANE: inner_radius=0이므로 d_eff = d (점 중심부터의 거리)
        double d_eff = d - inner_radius;
        // cost = cost_max * exp(-d_eff² / (2σ²))
        cost = cost_max * std::exp(inv_2sigma2 * d_eff * d_eff);
        // threshold 미만의 미세한 비용은 무시 → 계산량 절약
        if (cost < threshold) continue;
      }

      // Step 5d: Max-merge — 기존값과 새 비용 중 큰 값을 유지
      // 왜 sum이 아닌 max인가?
      //   → 두 콘 사이의 좁은 통로에서 비용이 합산되면 통과 불가능해짐
      //   → max-merge는 "가장 가까운 장애물까지의 비용"만 반영하므로
      //     좌우 경계 사이의 비용 골짜기가 자연스럽게 유지된다
      int idx = r * cols + c;
      if (cost > grid[idx]) {
        grid[idx] = cost;
      }
    }
  }
}

// ============================================================================
// generate — 메인 진입점: 좌/우 체인 → costmap 생성
// ============================================================================

/**
 * @brief costmap 생성 — 좌/우 체인 입력
 *
 * [전체 흐름]
 *
 *   1. 그리드 설정:
 *      - size_x × size_y 크기의 2D 격자를 resolution 단위로 분할
 *      - origin = (-size_x/2, -size_y/2) → ego 차량이 그리드 정중앙
 *
 *   2. 그리드 초기화:
 *      - 전체 셀을 0.0 (= 비용 없음 = 자유 공간)으로 초기화
 *
 *   3. 경계점 비용 적용:
 *      - 좌/우 체인의 각 ChainedPoint에 대해:
 *        - CONE → apply_source(cone_cost_max, sigma, threshold, cone_radius)
 *          → flat zone(cone_radius) + 가우시안 감쇠
 *        - LANE → apply_source(lane_cost_max, sigma, threshold, 0.0)
 *          → flat zone 없이 바로 가우시안 감쇠
 *
 *   결과적으로, costmap에는 경계점 근처에 높은 비용(= 밀어내는 힘),
 *   도로 중앙에 낮은 비용(= 주행 가능 공간)이 형성된다.
 *
 *   [비용 지형 단면도 예시 (좌측 콘 — 도로 중앙 — 우측 차선)]
 *
 *     cost
 *     100 ┤████                                    ████
 *         │    ████                            ████
 *      50 ┤        ████                    ████
 *         │            ████            ████
 *       0 ┤────────────────████████████────────────── y축
 *         좌측콘    ← 경로 탐색 영역 →    우측차선
 *
 *   MagneticPlanner는 이 비용 지형의 "골짜기"를 따라 경로를 생성한다.
 */
CostmapResult CostmapGenerator::generate(
  const std::vector<ChainedPoint> & left_chain,
  const std::vector<ChainedPoint> & right_chain,
  const PlanningParams & params)
{
  CostmapResult result;

  // ── 1. 그리드 크기/해상도/원점 설정 ──
  const auto & cm = params.costmap;
  result.resolution = cm.resolution;
  // cols: X축(전후 방향) 셀 수. 예) size_x=10, resolution=0.05 → cols=200
  result.cols = static_cast<int>(std::round(cm.size_x / cm.resolution));
  // rows: Y축(좌우 방향) 셀 수. 예) size_y=10, resolution=0.05 → rows=200
  result.rows = static_cast<int>(std::round(cm.size_y / cm.resolution));
  // origin: 그리드 좌하단의 월드 좌표 → ego 차량이 정중앙에 오도록 설정
  // 예) size_x=10 → origin_x = -5.0 → 그리드 X 범위: [-5.0, +5.0]
  result.origin_x = -cm.size_x / 2.0;
  result.origin_y = -cm.size_y / 2.0;

  // ── 2. 전체 그리드를 0.0 (자유 공간)으로 초기화 ──
  result.data.assign(result.rows * result.cols, 0.0);

  // ── 3. 체인의 각 점에 대해 비용장 적용 ──
  // 헬퍼 람다: 체인 하나를 순회하며 각 점의 타입에 따라 파라미터 분기
  auto apply_chain = [&](const std::vector<ChainedPoint> & chain) {
    for (const auto & pt : chain) {
      // ChainedPoint → Point2D 변환 (costmap은 타입 정보가 필요 없음)
      const Point2D src = pt.to_point2d();

      if (pt.type == PointType::CONE) {
        // ★ 콘(PE 드럼/교통 콘):
        //   - cone_cost_max (높은 비용) + cone_radius (flat zone)
        //   - 콘의 물리적 크기(직경 500mm) + 안전 마진을 고려하여
        //     cone_radius 이내는 cost_max 고정 (절대 진입 금지)
        //   - cone_radius 바깥부터 가우시안 감쇠 시작
        apply_source(
          result.data, result.rows, result.cols,
          result.resolution, result.origin_x, result.origin_y,
          src, cm.cone_cost_max, cm.sigma, cm.cost_threshold,
          cm.cone_radius);
      } else {
        // ★ 차선(LANE):
        //   - lane_cost_max (콘보다 낮은 비용)
        //   - inner_radius = 0.0 → flat zone 없음
        //   - 차선은 물리적 두께가 10cm에 불과하므로 별도 금지 구역 불필요
        //   - 점 중심에서 바로 가우시안 감쇠 시작
        //   - 차선을 약간 밟는 것은 콘 충돌보다 덜 위험 → 낮은 cost_max
        apply_source(
          result.data, result.rows, result.cols,
          result.resolution, result.origin_x, result.origin_y,
          src, cm.lane_cost_max, cm.sigma, cm.cost_threshold,
          0.0);  // inner_radius = 0 → flat zone 없음
      }
    }
  };

  // 좌측 경계 체인 적용 (콘/차선 혼합 가능)
  apply_chain(left_chain);
  // 우측 경계 체인 적용 (콘/차선 혼합 가능)
  apply_chain(right_chain);

  // costmap 생성 완료 표시
  result.valid = true;
  return result;
}

}  // namespace chaining_mr_ver
