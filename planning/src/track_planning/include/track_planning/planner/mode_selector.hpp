/**
 * @file mode_selector.hpp
 * @brief 플래닝 모드(DIRECT / ASTAR)를 결정하는 인라인 함수 헤더
 *
 * ## 역할
 * - 매 프레임마다 센터라인 직접 추종(DIRECT)과 A* 계획(ASTAR) 중 하나를 선택한다.
 * - 5가지 트리거 조건 중 하나라도 해당하면 ASTAR 모드로 전환한다.
 * - enable_astar == false이면 항상 DIRECT 모드를 반환한다.
 *
 * ## 5가지 ASTAR 트리거 조건
 *
 *  1. pair_valid == false
 *     - 좌/우 코리도 경계 폴리라인 쌍이 유효하지 않음
 *     - 경계 인식 실패 또는 포인트 수 부족
 *
 *  2. virtual_used == true
 *     - 가상 경계(virtual corridor)를 사용 중
 *     - 실제 경계 대신 추정값 사용 → 신뢰도 낮음
 *
 *  3. centerline_valid == false
 *     - 센터라인 계산 실패 (경계가 교차하거나 너무 짧음)
 *
 *  4. centerline_collision == true
 *     - 센터라인이 검증용 코스트맵의 장애물/경계와 충돌
 *     - CostmapValidationBuilder::check_centerline_collision()이 true
 *
 *  5. centerline_jump == true
 *     - 프레임 간 센터라인의 갑작스러운 불연속 감지
 *     - 이전 프레임과의 거리 차이가 임계값 초과
 *
 * ## 모드별 동작
 *  DIRECT: 센터라인을 그대로 경로로 사용 (빠르고 안정적)
 *  ASTAR:  DrivableMaskScanline + A* 플래너로 회피 경로 생성 (느리지만 장애물 회피)
 */

#ifndef TRACK_PLANNING__PLANNER__MODE_SELECTOR_HPP_
#define TRACK_PLANNING__PLANNER__MODE_SELECTOR_HPP_

#include "track_planning/common/types.hpp"

namespace track_planning
{
namespace mode_selector
{

/**
 * @brief 플래닝 모드 선택 함수 (인라인)
 *
 * ## 결정 로직
 *  if (!enable_astar)          → DIRECT  (A* 기능 자체가 비활성화됨)
 *  elif 5가지 조건 중 하나라도  → ASTAR   (예외 상황 발생)
 *  else                        → DIRECT  (정상 주행)
 *
 * @param pair_valid         좌/우 경계 폴리라인 쌍이 유효한지 여부
 * @param virtual_used       가상 경계 사용 중 여부
 * @param centerline_valid   센터라인이 유효한지 여부
 * @param centerline_collision 센터라인이 코스트맵과 충돌하는지 여부
 * @param centerline_jump    이전 프레임과의 센터라인 불연속 감지 여부
 * @param enable_astar       A* 모드 활성화 플래그 (false이면 항상 DIRECT)
 * @return                   PathMode::DIRECT 또는 PathMode::ASTAR
 */
inline PathMode select(
  bool pair_valid,
  bool virtual_used,
  bool centerline_valid,
  bool centerline_collision,
  bool centerline_jump,
  bool enable_astar)
{
  // A* 기능이 비활성화된 경우 항상 DIRECT 모드
  if (!enable_astar) {
    return PathMode::DIRECT;
  }

  // 5가지 ASTAR 트리거 조건 중 하나라도 해당하면 ASTAR 모드
  // - !pair_valid       : 경계 쌍이 유효하지 않음
  // - virtual_used      : 가상 경계 사용 중 (신뢰도 낮음)
  // - !centerline_valid : 센터라인 계산 실패
  // - centerline_collision : 센터라인이 장애물과 충돌
  // - centerline_jump   : 프레임 간 불연속 (갑작스러운 변화)
  if (!pair_valid || virtual_used || !centerline_valid ||
      centerline_collision || centerline_jump)
  {
    return PathMode::ASTAR;
  }

  // 모든 조건이 정상이면 DIRECT 모드 (빠른 센터라인 추종)
  return PathMode::DIRECT;
}

}  // namespace mode_selector
}  // namespace track_planning

#endif  // TRACK_PLANNING__PLANNER__MODE_SELECTOR_HPP_
