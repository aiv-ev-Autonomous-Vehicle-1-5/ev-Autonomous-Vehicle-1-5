/**
 * @file centerline_builder.hpp
 * @brief 코리도 경계에서 중심선(centerline)을 생성하는 클래스
 *
 * 파이프라인 Step (4): 좌/우 경계 폴리라인에서 주행 중심선을 계산한다.
 * 경계 유효성 상태에 따라 3가지 Case로 처리:
 *
 *   Case 1 (both_ok = true):
 *     - 좌/우 경계가 모두 유효 (corridor.left_ok && corridor.right_ok)
 *     - DTR 외심(circumcenter) 방식으로 센터라인 생성
 *
 *   Case 2 (virtual_used = true, both_ok = false):
 *     - 한쪽 경계만 실측, 반대쪽은 가상(virtual) 경계
 *     - 실측(visible) 경계에서 법선 방향으로 w_hat/2 만큼 오프셋
 *
 *   Case 3 (실패):
 *     - 어느 조건도 만족 안 함 → 빈 CenterlineResult 반환
 */
#ifndef TRACK_PLANNING__CORRIDOR__CENTERLINE_BUILDER_HPP_
#define TRACK_PLANNING__CORRIDOR__CENTERLINE_BUILDER_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <vector>

namespace track_planning
{

/**
 * @class CenterlineBuilder
 * @brief 좌/우 경계에서 주행 중심선을 생성
 *
 * 사용 예:
 *   CenterlineBuilder builder;
 *   CenterlineResult result = builder.build(
 *     left, right, both_ok, virtual_used, visible_is_left, w_hat, ds);
 *   if (result.valid) { // result.center 사용 }
 */
class CenterlineBuilder
{
public:
  /**
   * @brief 코리도 경계에서 중심선을 생성한다. (메인 진입점)
   *
   * 판단 우선순위:
   *   1. both_ok && left.size() >= 2 && right.size() >= 2 → Case 1 (DTR 외심)
   *   2. virtual_used && visible.size() >= 2               → Case 2 (offset)
   *   3. 그 외                                              → Case 3 (실패)
   */
  CenterlineResult build(
    const std::vector<Point2D> & left,
    const std::vector<Point2D> & right,
    bool both_ok,
    bool virtual_used,
    bool visible_is_left,
    double w_hat,
    const PlanningParams & params);

private:
  /// Case 1: 양쪽 유효 → DTR 외심 + 삼각형 인접 그래프 기반 센터라인
  CenterlineResult build_from_pair(
    const std::vector<Point2D> & left,
    const std::vector<Point2D> & right,
    const PlanningParams & params);

  /// Case 2: 한쪽(visible) + 가상 → 실측 경계에서 법선 방향 w_hat/2 오프셋
  CenterlineResult build_from_one_side(
    const std::vector<Point2D> & visible,
    bool visible_is_left,
    double w_hat,
    const PlanningParams & params);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__CORRIDOR__CENTERLINE_BUILDER_HPP_
