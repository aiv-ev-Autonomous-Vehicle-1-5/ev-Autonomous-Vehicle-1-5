/**
 * @file cdt_centerline.hpp
 * @brief CDT 기반 Centerline 추출기 — 좌/우 경계 체인으로부터 중심선 생성
 *
 * DTR 논문(Delaunay Triangulation-based Racing)에서 영감을 받아,
 * 좌/우 경계 체인에 Constrained Delaunay Triangulation(CDT)을 수행하고,
 * 삼각형의 외심(circumcenter)을 기하학적으로 필터링하여 centerline을 추출한다.
 *
 * ── 알고리즘 5단계 ──
 *   Step 1: 정점 + 제약 간선 구성
 *     - left/right component 점들을 CDT 정점으로 변환
 *     - 각 체인의 연속 쌍을 제약 간선(constraint edge)으로 등록
 *
 *   Step 2: CDT 수행
 *     - CDT::Triangulation으로 삼각형 분할
 *     - eraseOuterTrianglesAndHoles()로 제약 외부 삼각형 제거
 *
 *   Step 3: 외심(Circumcenter) 계산
 *     - 유효한 삼각형마다 외심 좌표를 계산
 *
 *   Step 4: 기하학적 필터링 (DTR 논문 heuristics)
 *     - Isosceles-like: 두 긴 변 비율 검사
 *     - Pointedness: 가장 긴 변 / 가장 짧은 변 비율 검사
 *     - Area: 최소 면적 검사
 *
 *   Step 5: Greedy Nearest-Neighbor Centerline 연결
 *     - ego(0,0)에 가장 가까운 외심에서 시작
 *     - 전방 검사(forward dot) + 최대 거리 조건으로 순차 연결
 *
 * ── 핵심 이점 ──
 *   - 격자(costmap) 없이 기하학적으로 중심선 추출 → 해상도 제약 없음
 *   - 계산량 감소: O(N log N) CDT vs O(N²) costmap 채우기
 *   - dead-end 방지: 삼각형 토폴로지로 연결성 보장
 */
#ifndef CHAINING_MR_VER__PLANNER__CDT_CENTERLINE_HPP_
#define CHAINING_MR_VER__PLANNER__CDT_CENTERLINE_HPP_

#include "chaining_mr_ver/common/types.hpp"

#include <vector>

namespace chaining_mr_ver
{

// Forward declaration — params.hpp를 include하지 않고 전방 선언
struct PlanningParams;

/**
 * @class CDTCenterlineExtractor
 * @brief 좌/우 경계 체인으로부터 CDT 기반 centerline을 추출하는 모듈
 *
 * CostmapGenerator + MagneticPlanner를 대체하는 단일 모듈.
 * DirectionChainer의 출력(component)을 받아 Point2D 벡터(centerline)를 반환한다.
 *
 * [파이프라인에서의 위치]
 *   Stage 2: DirectionChainer → Stage 3: CDTCenterlineExtractor → Stage 5: PostProcessor
 *
 * [Stateless 설계]
 *   멤버 상태를 유지하지 않는다. 매 콜백마다 extract()를 호출하면
 *   입력 데이터와 파라미터만으로 결과를 생성한다.
 */
class CDTCenterlineExtractor
{
public:
  /**
   * @brief 좌/우 경계 체인으로부터 CDT 기반 centerline 추출
   *
   * @param left_component   DirectionChainer가 출력한 좌측 경계 점들 (리샘플 완료)
   * @param right_component  DirectionChainer가 출력한 우측 경계 점들 (리샘플 완료)
   * @param params           플래너 파라미터 (cdt_planner 섹션 사용)
   * @return                 centerline을 구성하는 Point2D 벡터
   *                         (ego 근처에서 시작, 전방으로 진행)
   *                         입력이 부족하면 빈 벡터 반환
   */
  std::vector<Point2D> extract(
    const std::vector<ChainPoint> & left_component,
    const std::vector<ChainPoint> & right_component,
    const PlanningParams & params) const;
};

}  // namespace chaining_mr_ver

#endif  // CHAINING_MR_VER__PLANNER__CDT_CENTERLINE_HPP_
