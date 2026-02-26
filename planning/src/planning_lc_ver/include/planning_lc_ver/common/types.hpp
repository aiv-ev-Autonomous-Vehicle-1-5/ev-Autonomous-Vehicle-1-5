/**
 * @file types.hpp
 * @brief planning_lc_ver 패키지 전체에서 사용하는 공통 자료구조 정의
 *
 * planning_mr_ver 기반 + LineChainer를 위한 타입 확장:
 *   - PointType: 콘/차선 구분
 *   - ChainedPoint: 체이닝된 점 (타입 정보 포함)
 *   - ChainResult: 좌/우 체인 결과
 */
#ifndef PLANNING_LC_VER__COMMON__TYPES_HPP_
#define PLANNING_LC_VER__COMMON__TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace planning_lc_ver
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
 */
struct CostmapResult
{
  std::vector<double> data;   ///< row-major flat grid: data[row * cols + col]
  int rows = 0;
  int cols = 0;
  double resolution = 0.05;
  double origin_x = -5.0;
  double origin_y = -5.0;
  bool valid = false;
};

/**
 * @brief PathPostprocessor의 출력 결과를 담는 구조체
 */
struct PostprocessResult
{
  std::vector<Point2D> path;
  std::vector<double> yaw;
  bool valid = false;
};

/**
 * @brief 플래너의 현재 상태를 나타내는 열거형
 */
enum class PlannerState : uint8_t
{
  OK = 0,
  STOP = 1,
  INFEASIBLE = 2,
  STALE = 3
};

// ============================================================================
// LineChainer 확장 타입
// ============================================================================

/**
 * @brief 경계점의 원본 타입 (콘 vs 차선)
 *
 * costmap에서 콘은 cone_cost_max + cone_radius (flat zone),
 * 차선은 lane_cost_max (flat zone 없음)로 차별 적용된다.
 */
enum class PointType : uint8_t
{
  CONE = 0,  ///< LiDAR DBSCAN 결과 (PE 드럼/교통 콘)
  LANE = 1   ///< 카메라 차선 인식 결과
};

/**
 * @brief 체이닝된 경계점 — 위치 + 원본 타입 정보
 *
 * LineChainer가 출력하는 점. CostmapGenerator에서 type에 따라
 * 콘/차선 각각의 cost 파라미터를 적용한다.
 */
struct ChainedPoint
{
  double x = 0.0;
  double y = 0.0;
  PointType type = PointType::LANE;

  /// Point2D로 변환 (costmap/planner 모듈과의 호환용)
  Point2D to_point2d() const { return {x, y}; }
};

/**
 * @brief LineChainer의 출력 결과 — 좌/우 체인
 *
 * left_chain: 좌측 corridor (left seed에서 출발한 DFS chain)
 * right_chain: 우측 corridor (right seed에서 출발한 DFS chain)
 * 리샘플링까지 완료된 상태로 CostmapGenerator에 전달된다.
 */
struct ChainResult
{
  std::vector<ChainedPoint> left_chain;   ///< 좌측 경계 체인 (리샘플 완료)
  std::vector<ChainedPoint> right_chain;  ///< 우측 경계 체인 (리샘플 완료)
  bool valid = false;
};

}  // namespace planning_lc_ver

#endif  // PLANNING_LC_VER__COMMON__TYPES_HPP_
