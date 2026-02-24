/**
 * @file virtual_boundary.hpp
 * @brief 가상 경계(Virtual Boundary) 생성 클래스
 *
 * 파이프라인 Step (3): 한쪽 경계만 보일 때 (예: 한쪽 차선이 인식되지 않을 때)
 * 보이는(visible) 경계에서 반대쪽 가상 경계를 생성한다.
 *
 * 알고리즘:
 *   1. visible 폴리라인의 각 점에서 접선(tangent) 계산
 *   2. 접선을 90도 회전(rotate90)하여 법선(normal) 벡터 생성
 *      → rotate90(t): CCW 회전 → 좌측 법선 방향
 *   3. 부호(sgn) 결정:
 *      - visible이 좌측이면 → 가상 경계는 우측 → sgn = -1 (법선 반대 방향)
 *      - visible이 우측이면 → 가상 경계는 좌측 → sgn = +1 (법선 방향)
 *   4. 가상점 = visible[i] + sgn * w_hat * normal
 *   5. ROI 범위 밖 점은 제외 (부분적 가상 경계도 유용하므로 계속 진행)
 *
 * w_hat (추정 차로 폭) 관련:
 *   - EMA(지수 이동 평균)로 점진적 갱신: update_w_hat()
 *   - 초기화: 이전 쌍 검증 결과가 있으면 측정값 사용, 없으면 기본값: init_w_hat()
 */
#ifndef TRACK_PLANNING__CORRIDOR__VIRTUAL_BOUNDARY_HPP_
#define TRACK_PLANNING__CORRIDOR__VIRTUAL_BOUNDARY_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <vector>

namespace track_planning
{

/**
 * @class VirtualBoundary
 * @brief 보이는 한쪽 경계에서 반대쪽 가상 경계를 생성
 *
 * 사용 시나리오:
 *   - 좌측 차선은 인식되었으나 우측 차선이 없는 경우 (콘만 있는 구간 등)
 *   - 우측 경계만 있고 좌측이 없는 경우
 *   → 추정 차로 폭(w_hat)을 이용해 반대쪽 가상 경계 생성
 */
class VirtualBoundary
{
public:
  /**
   * @brief 보이는 경계에서 가상 반대쪽 경계를 생성한다.
   *
   * 처리 과정:
   *   1. visible 길이 및 최소 차로 폭 확인 (vehicle.width + safety.margin)
   *   2. polyline_tangents()로 visible 각 점의 접선 계산
   *   3. rotate90(tangent): CCW 90도 회전으로 좌측 법선 벡터 생성
   *   4. sgn = (visible_is_left ? -1.0 : +1.0)
   *      → 법선 방향 부호 결정 (가상 경계가 어느 쪽인지)
   *   5. p_virtual = visible[i] + sgn * w_hat * normal
   *   6. ROI 범위 내 점만 boundary에 추가
   *   7. boundary가 2점 이상이면 success = true
   *
   * @param visible          보이는 쪽의 경계 폴리라인
   * @param visible_is_left  true: visible이 좌측, 가상은 우측
   *                         false: visible이 우측, 가상은 좌측
   * @param w_hat            추정 차로 폭 [m] (EMA로 갱신됨)
   * @param p                플래닝 파라미터 (vehicle, safety, roi 포함)
   * @return                 VirtualBoundaryResult (boundary 점 배열, success 여부)
   */
  VirtualBoundaryResult generate(
    const std::vector<Point2D> & visible,
    bool visible_is_left,
    double w_hat,
    const PlanningParams & p);

  /**
   * @brief EMA(지수 이동 평균)로 w_hat을 갱신한다.
   *
   * 공식: w_hat_new = alpha * width_measured + (1 - alpha) * w_hat_prev
   *   - alpha 값이 크면 측정값에 빠르게 반응 (응답성 높음, 노이즈 민감)
   *   - alpha 값이 작으면 이전 추정값을 더 신뢰 (안정성 높음, 응답성 낮음)
   *
   * @param w_hat_prev      이전 추정 차로 폭 [m]
   * @param width_measured  이번 프레임에서 측정된 차로 폭 [m] (PairResult.width_median)
   * @param alpha           EMA 계수 (0 < alpha < 1)
   * @return                갱신된 w_hat [m]
   */
  static double update_w_hat(double w_hat_prev, double width_measured, double alpha);

  /**
   * @brief w_hat 초기값을 결정한다.
   *
   * 초기화 우선순위:
   *   1. 이전 쌍 검증이 유효(prev_pair_valid = true)하고
   *      측정 폭이 양수(prev_width_median > 0)이면 → 측정값 사용
   *   2. 그렇지 않으면 → default_width (파라미터에서 설정된 기본 차로 폭) 사용
   *
   * @param prev_width_median  이전 PairResult.width_median
   * @param prev_pair_valid    이전 PairResult.valid
   * @param default_width      파라미터 기본 차로 폭 [m]
   * @return                   초기 w_hat [m]
   */
  static double init_w_hat(
    double prev_width_median,
    bool prev_pair_valid,
    double default_width);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__CORRIDOR__VIRTUAL_BOUNDARY_HPP_
