/**
 * @file costmap_generator.hpp
 * @brief Magnetic Resistance Costmap 생성기 — LineChainer 체인 입력 버전
 *
 * ──────────────────────────────────────────────────────────────────
 * [자기 저항(Magnetic Resistance) Costmap 개념]
 *
 * 경계점(콘, 차선)을 "자석"처럼 취급하여, 각 점이 주변 공간에
 * "밀어내는 힘(반발력)"을 방사하는 비용 지도를 생성한다.
 * 이 비용 지도 위에서 경로 탐색기(MagneticPlanner)가 동작하면,
 * 경계점에 가까울수록 비용이 높아져 자연스럽게 경계를 회피하는
 * 경로가 생성된다.
 *
 * 물리적 비유:
 *   - 각 경계점 = N극 자석
 *   - 차량(ego) = N극 자석
 *   - 같은 극끼리 밀어내므로, 차량은 경계점에서 멀어지려 한다
 *   - 좌/우 경계 사이의 비용이 가장 낮은 "골짜기"가 주행 경로가 된다
 *
 * [가우시안 비용 공식]
 *
 *   cost(d) = cost_max * exp( -d² / (2 * sigma²) )
 *
 *   - d: 경계점으로부터의 유클리드 거리 [m]
 *   - cost_max: 경계점 바로 위에서의 최대 비용 (정점)
 *   - sigma: 가우시안 표준편차 — 비용이 퍼지는 정도를 결정
 *     (sigma가 크면 넓게 약하게, 작으면 좁게 강하게 퍼짐)
 *
 *   그래프 모양:
 *     cost_max ┤███
 *              │ ████
 *              │   ████
 *     threshold┤─────████──────── (이 이하의 비용은 무시)
 *              │       ████
 *           0  └──────────────── d
 *              0   σ   2σ   3σ
 *
 * [콘(CONE)의 Flat Zone]
 *
 *   PE 드럼/교통 콘은 물리적 크기가 있으므로 (직경 500mm),
 *   중심으로부터 cone_radius(= ~0.65m) 이내는 "절대 진입 금지 구역"으로
 *   설정한다. 이 구간에서는 거리에 관계없이 cost_max를 유지한다:
 *
 *     d <= cone_radius  → cost = cost_max  (flat zone, 무조건 최대)
 *     d >  cone_radius  → cost = cost_max * exp(-(d-cone_radius)² / (2σ²))
 *
 *   반면 차선(LANE) 점은 물리적 크기가 0이므로 inner_radius=0,
 *   중심에서 바로 가우시안 감쇠가 시작된다.
 *
 * [그리드 좌표계]
 *
 *   costmap은 ego 차량(base_link)을 중심으로 한 2D 격자다:
 *
 *     origin = (-size_x/2, -size_y/2)  ← 그리드 좌하단의 월드 좌표
 *     cols = size_x / resolution        ← X축 셀 개수
 *     rows = size_y / resolution        ← Y축 셀 개수
 *
 *   셀 (r, c)의 월드 좌표:
 *     wx = origin_x + (c + 0.5) * resolution
 *     wy = origin_y + (r + 0.5) * resolution
 *     (+0.5는 셀의 중심점을 가리키기 위함)
 *
 *   ego 차량은 그리드의 정중앙에 위치한다.
 *
 * ──────────────────────────────────────────────────────────────────
 *
 * 기존 mr_ver에서는 cones/lanes를 별도 벡터로 받았지만,
 * lc_ver에서는 LineChainer가 출력한 좌/우 ChainedPoint 체인을 직접 받는다.
 * ChainedPoint.type에 따라 콘/차선 각각의 cost 파라미터를 적용한다.
 */
#ifndef PLANNING_LC_VER__COSTMAP__COSTMAP_GENERATOR_HPP_
#define PLANNING_LC_VER__COSTMAP__COSTMAP_GENERATOR_HPP_

#include "planning_lc_ver/common/types.hpp"
#include "planning_lc_ver/common/params.hpp"
#include <vector>

namespace planning_lc_ver
{

/**
 * @class CostmapGenerator
 * @brief 체이닝된 경계점들로부터 2D 비용 그리드(costmap)를 생성하는 클래스
 *
 * [파이프라인 위치]
 *   DirectionChainer → [CostmapGenerator] → MagneticPlanner
 *
 * [입력] 좌/우 ChainedPoint 체인 (각 점에 CONE/LANE 타입 정보 포함)
 * [출력] CostmapResult (row-major flat 배열, 각 셀에 0.0~cost_max 범위의 비용값)
 *
 * [비용 병합 규칙]
 *   여러 경계점의 비용 영역이 겹치는 셀에서는 "최대값(max-merge)"을 사용한다.
 *   (합산(sum)이 아님! 두 콘 사이에서 비용이 비정상적으로 폭증하는 것을 방지)
 */
class CostmapGenerator
{
public:
  /**
   * @brief 좌/우 체인으로부터 costmap을 생성하는 메인 인터페이스
   *
   * @param left_chain  좌측 경계 체인 (리샘플 완료, ChainedPoint 타입 포함)
   * @param right_chain 우측 경계 체인
   * @param params      파라미터 (costmap 섹션의 size, resolution, sigma 등)
   * @return CostmapResult 생성된 costmap 그리드
   *
   * [처리 순서]
   *   1. 파라미터로부터 그리드 크기/해상도/원점 계산
   *   2. 전체 셀을 0.0으로 초기화
   *   3. left_chain의 각 점에 대해 apply_source() 호출 (CONE/LANE 분기)
   *   4. right_chain의 각 점에 대해 apply_source() 호출
   *   5. 결과 반환
   */
  CostmapResult generate(
    const std::vector<ChainedPoint> & left_chain,
    const std::vector<ChainedPoint> & right_chain,
    const PlanningParams & params);

private:
  /**
   * @brief 단일 경계점이 방사하는 가우시안 비용장을 그리드에 적용
   *
   * 하나의 "자석(source)"이 주변 셀에 비용을 뿌리는 핵심 함수.
   * inner_radius > 0이면 그 안쪽은 flat zone(cost_max 고정),
   * 그 바깥부터 가우시안 감쇠가 시작된다.
   *
   * @param grid         비용 그리드 (row-major flat 배열, 이 함수가 직접 수정)
   * @param rows         그리드 행 수
   * @param cols         그리드 열 수
   * @param resolution   셀 한 변의 크기 [m]
   * @param origin_x     그리드 좌하단의 월드 X 좌표 [m]
   * @param origin_y     그리드 좌하단의 월드 Y 좌표 [m]
   * @param source       비용을 방사하는 경계점의 월드 좌표 [m]
   * @param cost_max     경계점 중심(또는 flat zone 내)에서의 최대 비용
   * @param sigma        가우시안 표준편차 — 비용이 퍼지는 폭을 결정 [m]
   * @param threshold    이 값 이하의 비용은 무시 (계산량 절약)
   * @param inner_radius flat zone 반경 [m] (CONE: cone_radius, LANE: 0.0)
   *
   * [비용 병합]
   *   기존 셀 값과 새 비용 중 큰 값을 취한다 (max-merge).
   *   이렇게 하면 두 콘 사이의 좁은 통로에서도 비용이 합산되지 않아
   *   플래너가 통과할 수 있는 경로를 찾을 수 있다.
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
   * @brief 비용이 threshold 이상인 최대 거리를 계산 (가우시안 역함수)
   *
   * cost(r) = cost_max * exp(-r² / (2σ²)) >= threshold
   * 를 만족하는 최대 r을 구한다:
   *   r = sigma * sqrt(2 * ln(cost_max / threshold))
   *
   * 이 반경 바깥의 셀은 어차피 threshold 미만이므로 계산을 건너뛸 수 있다.
   * 이를 통해 전체 그리드를 순회하지 않고, source 주변의 일부 셀만
   * 처리하여 계산량을 크게 줄인다.
   *
   * @param cost_max   최대 비용
   * @param sigma      가우시안 표준편차 [m]
   * @param threshold  최소 유효 비용
   * @return double    유효 반경 [m] (이 거리 밖은 threshold 미만)
   */
  static double effective_radius(
    double cost_max, double sigma, double threshold);
};

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__COSTMAP__COSTMAP_GENERATOR_HPP_
