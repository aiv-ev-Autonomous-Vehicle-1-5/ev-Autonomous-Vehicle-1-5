/**
 * @file pair_validator.hpp
 * @brief 좌/우 경계 폴리라인 쌍의 유효성을 검증하는 클래스
 *
 * 파이프라인 Step (2): CorridorBuilder가 생성한 좌/우 경계 쌍이
 * 실제 차로를 올바르게 나타내는지 검증한다.
 *
 * 검증 항목:
 *   1. 리샘플링: 양쪽 폴리라인을 균일 간격으로 리샘플링
 *   2. 접선 비교: 각 점에서 좌/우 접선의 방향 편차 계산
 *   3. 부호 있는 폭(signed width): cross product로 좌/우가 역전되었는지 확인
 *   4. 교차(crossing) 검사: 폭이 음수이면 경계가 교차됨 → 무효
 *   5. 각도 통계: 평균/최대 각도 편차가 임계값 초과 시 무효
 *   6. 폭 통계: 중앙값이 [w_min, w_max] 범위 이탈 또는 표준편차 과다 시 무효
 */
#ifndef TRACK_PLANNING__CORRIDOR__PAIR_VALIDATOR_HPP_
#define TRACK_PLANNING__CORRIDOR__PAIR_VALIDATOR_HPP_

#include "track_planning/common/types.hpp"
#include "track_planning/common/params.hpp"

#include <vector>

namespace track_planning
{

/**
 * @class PairValidator
 * @brief 좌/우 경계 폴리라인 쌍이 유효한 주행 가능 코리도를 구성하는지 검증
 *
 * 사용 예:
 *   PairValidator validator;
 *   PairResult result = validator.validate(left_poly, right_poly, params);
 *   if (result.valid) { ... }
 */
class PairValidator
{
public:
  /**
   * @brief 좌/우 폴리라인 쌍의 유효성을 검증한다.
   *
   * 처리 순서:
   *   1. 양쪽 폴리라인을 resample_ds 간격으로 균일 리샘플링
   *   2. 각 점에서 접선 벡터 계산 → 좌/우 접선 방향 편차(dtheta) 수집
   *   3. 부호 있는 폭 계산: w = cross(tL, right - left)
   *      - w > 0: 올바른 배치 (우측이 좌측 진행 방향 기준 오른쪽)
   *      - w < 0: 경계 교차 → 즉시 무효 반환
   *   4. 각도 통계(angle_mean, angle_max) → 임계값 초과 시 무효
   *   5. 폭 통계(width_median, width_std) → 범위/편차 이탈 시 무효
   *   6. 모든 검사 통과 시 result.valid = true
   *
   * @param left   좌측 경계 폴리라인
   * @param right  우측 경계 폴리라인
   * @param p      플래닝 파라미터 (pair 관련 파라미터 포함)
   * @return       PairResult (valid, width_median, width_std, angle_mean)
   */
  PairResult validate(
    const std::vector<Point2D> & left,
    const std::vector<Point2D> & right,
    const PlanningParams & p);
};

}  // namespace track_planning

#endif  // TRACK_PLANNING__CORRIDOR__PAIR_VALIDATOR_HPP_
