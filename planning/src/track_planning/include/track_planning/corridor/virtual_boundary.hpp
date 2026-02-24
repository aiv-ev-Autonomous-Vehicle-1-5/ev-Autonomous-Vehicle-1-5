/**
 * @file virtual_boundary.hpp
 * @brief 가상 경계(Virtual Boundary) 생성 클래스
 *
 * 파이프라인 Step (3): 한쪽 경계만 보일 때
 * 보이는(visible) 경계에서 반대쪽 가상 경계를 생성한다.
 *
 * 알고리즘:
 *   1. visible 폴리라인의 각 점에서 접선(tangent) 계산
 *   2. 접선을 90도 회전(rotate90)하여 법선(normal) 벡터 생성
 *   3. 부호(sgn) 결정:
 *      - visible이 좌측이면 → 가상 경계는 우측 → sgn = -1
 *      - visible이 우측이면 → 가상 경계는 좌측 → sgn = +1
 *   4. 가상점 = visible[i] + sgn * w_hat * normal
 *   5. ROI 범위 밖 점은 제외
 *
 * w_hat (추정 차로 폭):
 *   - 양쪽 corridor가 모두 보일 때 median_width 직접 계산 → w_hat_에 저장
 *   - 한쪽만 보일 때 직전 프레임의 w_hat_ 사용
 *   - 초기값: default_track_width (1.5m)
 */
#ifndef TRACK_PLANNING__CORRIDOR__VIRTUAL_BOUNDARY_HPP_
#define TRACK_PLANNING__CORRIDOR__VIRTUAL_BOUNDARY_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <vector>

namespace track_planning
{

class VirtualBoundary
{
public:
  /**
   * @brief 보이는 경계에서 가상 반대쪽 경계를 생성한다.
   *
   * @param visible          보이는 쪽의 경계 폴리라인
   * @param visible_is_left  true: visible이 좌측, 가상은 우측
   * @param w_hat            추정 차로 폭 [m]
   * @param p                플래닝 파라미터 (vehicle, safety, roi 포함)
   * @return                 VirtualBoundaryResult (boundary 점 배열, success 여부)
   */
  VirtualBoundaryResult generate(
    const std::vector<Point2D> & visible,
    bool visible_is_left,
    double w_hat,
    const PlanningParams & p);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__CORRIDOR__VIRTUAL_BOUNDARY_HPP_
