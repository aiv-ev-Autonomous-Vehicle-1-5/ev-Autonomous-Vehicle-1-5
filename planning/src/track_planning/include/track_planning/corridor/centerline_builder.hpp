/**
 * @file centerline_builder.hpp
 * @brief 코리도 경계에서 중심선(centerline)을 생성하는 클래스
 *
 * 파이프라인 Step (4): 좌/우 경계 폴리라인에서 주행 중심선을 계산한다.
 * 경계 유효성 상태에 따라 3가지 Case로 처리:
 *
 *   Case 1 (pair_valid = true):
 *     - 좌/우 경계가 모두 유효한 쌍을 이룸
 *     - 두 경계를 균일 리샘플링 후 대응점 중간값(midpoint) 평균
 *     - center[i] = 0.5 * (left[i] + right[i])
 *     - 가장 정확한 방법
 *
 *   Case 2 (virtual_used = true, pair_valid = false):
 *     - 한쪽 경계만 실측, 반대쪽은 가상(virtual) 경계
 *     - 실측(visible) 경계에서 법선 방향으로 w_hat/2 만큼 오프셋
 *     - sgn * (w_hat / 2) * rotate90(tangent): 법선 방향 절반 폭 이동
 *     - w_hat 정확도에 따라 오차 발생 가능
 *
 *   Case 3 (실패):
 *     - 어느 조건도 만족 안 함 → 빈 CenterlineResult 반환
 */
#ifndef TRACK_PLANNING__CORRIDOR__CENTERLINE_BUILDER_HPP_
#define TRACK_PLANNING__CORRIDOR__CENTERLINE_BUILDER_HPP_

#include "track_planning/common/types.hpp"

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
 *     left, right, pair_valid, virtual_used, visible_is_left, w_hat, ds);
 *   if (result.valid) { // result.center 사용 }
 */
class CenterlineBuilder
{
public:
  /**
   * @brief 코리도 경계에서 중심선을 생성한다. (메인 진입점)
   *
   * 판단 우선순위:
   *   1. pair_valid && left.size() >= 2 && right.size() >= 2 → Case 1 (midpoint)
   *   2. virtual_used && visible.size() >= 2                 → Case 2 (offset)
   *   3. 그 외                                               → Case 3 (실패)
   *
   * @param left           좌측 경계 폴리라인 (실측 또는 가상)
   * @param right          우측 경계 폴리라인 (실측 또는 가상)
   * @param pair_valid     PairValidator 검증 통과 여부 (두 경계가 유효한 쌍)
   * @param virtual_used   가상 경계 사용 여부 (한쪽이 VirtualBoundary에서 생성됨)
   * @param visible_is_left  (virtual_used 시) 실측 경계가 좌측이면 true
   * @param w_hat          추정 차로 폭 [m] (Case 2에서 오프셋 거리로 사용)
   * @param resample_ds    출력 중심선 리샘플링 간격 [m]
   * @return               CenterlineResult (center 폴리라인, valid 여부)
   */
  CenterlineResult build(
    const std::vector<Point2D> & left,
    const std::vector<Point2D> & right,
    bool pair_valid,
    bool virtual_used,
    bool visible_is_left,
    double w_hat,
    double resample_ds);

private:
  /**
   * @brief Case 1: 양쪽 유효 → 대응점 중간값 평균으로 중심선 생성
   *
   * 처리 과정:
   *   1. 양쪽을 resample_ds 간격으로 균일 리샘플링
   *   2. 짧은 쪽 길이 n 기준으로 대응점 쌍 설정
   *   3. center[i] = {0.5*(left[i].x + right[i].x), 0.5*(left[i].y + right[i].y)}
   *   4. 결과를 다시 resample_ds 간격으로 리샘플링 (균일 출력 보장)
   *
   * @param left        좌측 경계 폴리라인
   * @param right       우측 경계 폴리라인
   * @param resample_ds 리샘플링 간격 [m]
   * @return            CenterlineResult
   */
  CenterlineResult build_from_pair(
    const std::vector<Point2D> & left,
    const std::vector<Point2D> & right,
    double resample_ds);

  /**
   * @brief Case 2: 한쪽(visible) + 가상 → 실측 경계에서 법선 방향 w_hat/2 오프셋
   *
   * 처리 과정:
   *   1. visible을 resample_ds 간격으로 리샘플링
   *   2. 각 점의 접선 계산 → rotate90으로 좌측 법선 생성
   *   3. 부호 결정:
   *      - visible이 좌측(LEFT): 중심은 오른쪽 → sgn = -1
   *      - visible이 우측(RIGHT): 중심은 왼쪽 → sgn = +1
   *   4. center[i] = visible_rs[i] + sgn * (w_hat/2) * rotate90(tangent[i])
   *   5. 결과를 다시 resample_ds 간격으로 리샘플링
   *
   * @param visible         실측 경계 폴리라인
   * @param visible_is_left 실측 경계가 좌측이면 true
   * @param w_hat           추정 차로 폭 [m]
   * @param resample_ds     리샘플링 간격 [m]
   * @return                CenterlineResult
   */
  CenterlineResult build_from_one_side(
    const std::vector<Point2D> & visible,
    bool visible_is_left,
    double w_hat,
    double resample_ds);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__CORRIDOR__CENTERLINE_BUILDER_HPP_
