/**
 * @file costmap_generator.hpp
 * @brief 가우시안 비용 지도(Costmap) 생성기
 *
 * 경계점(콘, 차선)의 각 점에서 주변 공간으로
 * 가우시안 비용을 방사하는 2D 격자 비용 지도를 생성한다.
 *
 * ── 비용 모델 ──
 *
 *   [가우시안 비용 공식]
 *     cost(d) = cost_max * exp( -d² / (2 * sigma²) )
 *
 *     d=0 (점 위)에서 cost_max, 거리가 멀어질수록 지수적 감소.
 *     sigma가 클수록 넓게 퍼지고, 작을수록 좁고 날카로운 비용 장벽.
 *
 *   [CONE vs LANE 차등 비용]
 *     ┌─────────────┬──────────────┬───────────┬────────────────────────┐
 *     │ 타입        │ cost_max     │ flat zone │ 의미                    │
 *     ├─────────────┼──────────────┼───────────┼────────────────────────┤
 *     │ CONE (콘)   │ 100 (높음)   │ cone_radius(0.65m) │ 강한 회피 (물리적 장벽) │
 *     │ LANE (차선) │  50 (낮음)   │ 없음(0m)  │ 넘을 수 있음 (비용만 추가) │
 *     └─────────────┴──────────────┴───────────┴────────────────────────┘
 *
 *     이 차이로 A*가:
 *     - 콘은 절대 통과 불가 (obstacle_cost=100이면 벽)
 *     - 차선은 비용을 감수하고 넘을 수 있음 (차선→콘 트랙 전환 시 유용)
 *
 *   [flat zone (내부 반경)]
 *     콘의 물리적 크기(PE 드럼 직경 500mm) 이내에서는
 *     가우시안 감쇠 없이 cost_max를 유지한다.
 *     → 콘 중심부에 "평탄한 최대비용 원판"이 생김.
 *     → flat zone 바깥부터 가우시안 감쇠가 시작.
 *
 *   [unchained 포인트]
 *     체이닝에 실패한 점들은 정체를 알 수 없으므로
 *     보수적으로 CONE 비용(cone_cost_max + cone_radius)으로 처리.
 *
 * ── entry wall ──
 *
 *   ego(차량) 양옆에서 좌/우 시드까지 가상 콘 벽을 세워서,
 *   A*가 시드 사이의 "입구"로만 진입하도록 유도한다.
 *   이 벽이 없으면 A*가 경계 뒤쪽으로 돌아가는 경로를 생성할 수 있다.
 *
 *     좌측 벽: ego(0, +entry_wall_ego_y) → left_seed
 *     우측 벽: ego(0, -entry_wall_ego_y) → right_seed
 *
 *     ┌─── costmap ────────────┐
 *     │  ■■■■■  left_seed      │  ■ = 가상 콘 벽
 *     │ ■                      │
 *     │ ego ─── 입구 ──→ goal  │
 *     │ ■                      │
 *     │  ■■■■■  right_seed     │
 *     └────────────────────────┘
 */
#ifndef CHAINING_COSTMAP_VER__COSTMAP__COSTMAP_GENERATOR_HPP_
#define CHAINING_COSTMAP_VER__COSTMAP__COSTMAP_GENERATOR_HPP_

#include "chaining_costmap_ver/common/types.hpp"
#include "chaining_costmap_ver/common/params.hpp"
#include <vector>

namespace chaining_costmap_ver
{

class CostmapGenerator
{
public:
  /**
   * @brief 좌/우 체인 + unchained 포인트로 costmap 생성
   *
   * 각 ChainedPoint의 type(CONE/LANE)에 따라 비용 파라미터를 다르게 적용.
   * 여러 source가 겹치는 영역은 max-merge (큰 값 유지).
   *
   * @param left_chain   좌측 경계 체인 (리샘플 완료된 균등 간격 점열)
   * @param right_chain  우측 경계 체인
   * @param unchained    체이닝 실패 포인트 (CONE 비용으로 보수적 처리)
   * @param params       Costmap 파라미터 (size, resolution, sigma 등)
   * @return CostmapResult (격자 데이터 + 메타정보)
   */
  CostmapResult generate(
    const std::vector<ChainedPoint> & left_chain,
    const std::vector<ChainedPoint> & right_chain,
    const std::vector<ChainedPoint> & unchained,
    const PlanningParams & params);

  /**
   * @brief 시드→ego 양옆까지 cone_cost_max 벽을 그려서 입구로 유도
   *
   * 좌/우 시드에서 ego(x=0) 양옆까지 가상 콘을 일정 간격으로 샘플링하여
   * costmap에 비용 장벽을 그린다. A*가 시드 사이(입구)로만 진입하게 한다.
   *
   * @param costmap      generate()로 생성된 costmap (in-place 수정)
   * @param left_seed    좌측 체인의 시드 위치
   * @param right_seed   우측 체인의 시드 위치
   * @param params       costmap 파라미터 (entry_wall_ego_y, cone_cost_max 등)
   */
  static void apply_entry_walls(
    CostmapResult & costmap,
    const Point2D & left_seed,
    const Point2D & right_seed,
    const PlanningParams & params);

private:
  /**
   * @brief 단일 경계점의 가우시안 비용장을 그리드에 적용 (max-merge)
   *
   * source 점을 중심으로 effective_radius 범위 내의 셀만 순회하여
   * 가우시안 비용을 계산하고, 기존 값보다 클 때만 갱신한다.
   *
   * 비용 프로파일:
   *   d ≤ inner_radius : cost = cost_max (flat zone)
   *   d > inner_radius : cost = cost_max * exp(-(d-inner_radius)² / (2σ²))
   *
   * @param grid         비용 격자 데이터 (rows × cols, double[])
   * @param rows, cols   격자 크기
   * @param resolution   셀 크기 [m/cell]
   * @param origin_x/y   격자 원점 월드 좌표 [m]
   * @param source       비용을 방사할 중심점 (월드 좌표)
   * @param cost_max     최대 비용 (cone_cost_max 또는 lane_cost_max)
   * @param sigma        가우시안 표준편차 [m]
   * @param threshold    비용 컷오프 (이 이하는 0으로 처리 → 연산량 절감)
   * @param inner_radius flat zone 반경 [m] (CONE: cone_radius, LANE: 0)
   */
  static void apply_source(
    std::vector<double> & grid,
    int rows, int cols,
    double resolution,
    double origin_x, double origin_y,
    const Point2D & source,
    double cost_max,
    double sigma,
    double threshold,
    double inner_radius);

  /**
   * @brief 비용이 threshold 이상인 최대 거리 계산 (가우시안 역함수)
   *
   * 가우시안 cost(d) = cost_max * exp(-d²/(2σ²)) = threshold 를 풀면:
   *   d = σ * √(2 * ln(cost_max / threshold))
   *
   * 이 거리 바깥에서는 비용이 threshold 미만이므로 순회할 필요 없다.
   * apply_source()에서 순회 범위를 제한하여 O(N²) 전체 격자 순회를 피한다.
   *
   * @param cost_max   최대 비용
   * @param sigma      가우시안 표준편차 [m]
   * @param threshold  비용 컷오프
   * @return 유효 반경 [m] (이 거리 밖에서는 비용 < threshold)
   */
  static double effective_radius(
    double cost_max, double sigma, double threshold);
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__COSTMAP__COSTMAP_GENERATOR_HPP_
