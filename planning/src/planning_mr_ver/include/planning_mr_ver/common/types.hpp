/**
 * @file types.hpp
 * @brief planning_mr_ver 패키지 전체에서 사용하는 공통 자료구조 정의
 *
 * 이 헤더는 MR(Magnetic Resistance) 플래너 파이프라인의 모든 모듈이
 * 공유하는 기본 타입(Point2D, CostmapResult, PostprocessResult, PlannerState)을 정의한다.
 */
#ifndef PLANNING_MR_VER__COMMON__TYPES_HPP_
#define PLANNING_MR_VER__COMMON__TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace planning_mr_ver
{

/**
 * @brief 2D 좌표를 나타내는 기본 점 구조체
 *
 * ego 차량 좌표계(base_link) 기준:
 *   x = 전방(+) / 후방(-)
 *   y = 좌측(+) / 우측(-)
 */
struct Point2D
{
  double x = 0.0;  ///< X 좌표 [m]
  double y = 0.0;  ///< Y 좌표 [m]
};

/**
 * @brief CostmapGenerator의 출력 결과를 담는 구조체
 *
 * ego(0,0) 중심으로 size_x × size_y 크기의 2D 그리드를 생성한다.
 * data는 row-major 1차원 배열이며, data[row * cols + col]로 접근한다.
 *
 * 그리드 좌표 ↔ 월드 좌표 변환:
 *   world_x = origin_x + (col + 0.5) * resolution
 *   world_y = origin_y + (row + 0.5) * resolution
 *
 * 각 셀의 값 범위: 0.0 ~ 100.0
 *   0   = 장애물 없음 (안전)
 *   100 = 콘 중심 (가장 위험)
 *   50  = 차선 중심
 */
struct CostmapResult
{
  std::vector<double> data;   ///< row-major flat grid: data[row * cols + col]
  int rows = 0;               ///< 그리드 행 수 (Y축 방향 셀 개수)
  int cols = 0;               ///< 그리드 열 수 (X축 방향 셀 개수)
  double resolution = 0.05;   ///< 셀 크기 [m/cell] (기본: 5cm → 200×200 그리드)
  double origin_x = -5.0;     ///< cell(0,0)의 월드 X 좌표 [m] (그리드 좌하단)
  double origin_y = -5.0;     ///< cell(0,0)의 월드 Y 좌표 [m] (그리드 좌하단)
  bool valid = false;          ///< 생성 성공 여부 플래그
};

/**
 * @brief PathPostprocessor의 출력 결과를 담는 구조체
 *
 * prune → smooth → resample → yaw 계산 4단계를 거친 최종 경로.
 * path와 yaw는 같은 인덱스끼리 대응한다 (path[i]의 heading = yaw[i]).
 */
struct PostprocessResult
{
  std::vector<Point2D> path;  ///< 후처리된 경로 점 목록 [m]
  std::vector<double> yaw;    ///< 각 경로 점의 heading 각도 [rad] (atan2 기준)
  bool valid = false;          ///< 후처리 성공 여부 플래그
};

/**
 * @brief 플래너의 현재 상태를 나타내는 열거형
 *
 * 이 값은 /planning/status 토픽으로 퍼블리시되어
 * control 노드가 차량 제어 판단에 사용한다.
 */
enum class PlannerState : uint8_t
{
  OK = 0,          ///< 정상 — 경로 추종 가능
  STOP = 1,        ///< 정지 — 유효한 경로 없음
  INFEASIBLE = 2,  ///< 실행 불가 — 곡률이 차량 최소 회전반경 초과
  STALE = 3        ///< 입력 데이터 타임아웃 — perception 데이터 수신 안 됨
};

}  // namespace planning_mr_ver

#endif  // PLANNING_MR_VER__COMMON__TYPES_HPP_
