/**
 * @file planner_types.hpp
 * @brief 플래너 상태 및 후처리 결과 타입
 *
 * PathPostprocessor → SafetyChecker → 제어기 사이의 인터페이스 타입.
 *   - PostprocessResult: 후처리된 경로 + yaw 배열
 *   - PlannerState:      플래너 상태 열거형 (OK/FAIL/STALE/WARNING)
 */
#ifndef CHAINING_COSTMAP_VER__COMMON__TYPES__PLANNER_TYPES_HPP_
#define CHAINING_COSTMAP_VER__COMMON__TYPES__PLANNER_TYPES_HPP_

#include "chaining_costmap_ver/common/types/point_types.hpp"

#include <cstdint>
#include <vector>

namespace chaining_costmap_ver
{

/**
 * @brief PathPostprocessor의 출력 결과를 담는 구조체
 *
 * A* Planner가 출력한 raw 경로의 후처리 결과:
 *   1) Prune → 2) Smooth → 2.5) Curvature Clamp → 3) Resample → 4) Yaw
 *
 * path[i]와 yaw[i]는 1:1 대응.
 * yaw[i]는 path[i]에서 path[i+1] 방향의 헤딩 각도(rad).
 */
struct PostprocessResult
{
  std::vector<Point2D> path;  ///< 후처리 완료된 경로 좌표 [m]
  std::vector<double> yaw;    ///< 각 경로점의 헤딩 [rad] (-π ~ +π)
  bool valid = false;          ///< true: 후처리 성공

  std::vector<Point2D> pruned;  ///< prune 직후 결과 (디버깅용)
};

/**
 * @brief 플래너의 현재 상태를 나타내는 열거형
 *
 * SafetyChecker가 판단하여 제어기에 전달하는 상태.
 * /planning/status 토픽으로 발행되는 문자열:
 *   "OK"                    — 정상
 *   "STALE"                 — 센서 데이터 만료
 *   "FAIL - <reason>"       — 실패 + 원인
 *   "WARNING - <reason>"    — 경고 (경로는 발행하되 주의 필요)
 */
enum class PlannerState : uint8_t
{
  OK = 0,          ///< 정상 — 경로 추종 가능
  FAIL = 1,        ///< 실패 — 경로 생성/추종 불가
  STALE = 2,       ///< 데이터 만료 — 센서 입력 timeout
  WARNING = 3      ///< 경고 — 경로 추종 가능하나 주의 필요 (예: 곡률 초과)
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__COMMON__TYPES__PLANNER_TYPES_HPP_
