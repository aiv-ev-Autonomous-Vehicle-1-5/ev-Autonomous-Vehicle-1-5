/**
 * @file point_types.hpp
 * @brief 기본 좌표 및 경계점 타입 정의
 *
 * 패키지 전반에서 사용하는 최소 단위 구조체:
 *   - Point2D:      순수 2D 좌표
 *   - PointType:    bbox/차선 구분 열거형
 *   - ChainedPoint: 체이닝된 경계점 (위치 + 타입 + is_backbone 플래그)
 *                   is_backbone=true이면 costmap에서 타입 무관하게 bbox_cost_max 적용
 *   - ChainPoint:   DirectionChainer 입력 포인트 (위치 + 메타정보 + is_backbone 플래그)
 *                   to_chained_point() 변환 시 is_backbone을 ChainedPoint로 전파
 */
#ifndef CHAINING_COSTMAP_VER__COMMON__TYPES__POINT_TYPES_HPP_
#define CHAINING_COSTMAP_VER__COMMON__TYPES__POINT_TYPES_HPP_

#include <cstdint>
#include <vector>

namespace chaining_costmap_ver
{

// ============================================================================
// 기본 타입 (Basic Types)
// ============================================================================

/**
 * @brief 2D 좌표를 나타내는 기본 점 구조체
 *
 * ego 차량 좌표계(base_link) 기준:
 *   x = 전방(+) / 후방(-)
 *   y = 좌측(+) / 우측(-)
 *
 * CostmapGenerator, AStarPlanner, PathPostprocessor 등
 * "센서 타입에 무관하게" 좌표만 다루는 모듈에서 사용됨.
 */
struct Point2D
{
  double x = 0.0;  ///< X 좌표 [m] — 양수가 차량 전방, 음수가 후방
  double y = 0.0;  ///< Y 좌표 [m] — 양수가 좌측, 음수가 우측 (ROS 좌표계 관례)
};

// ============================================================================
// 경계점 타입 (Boundary Point Types)
// ============================================================================

/**
 * @brief 경계점의 원본 타입 (bbox vs 차선)
 *
 * costmap에서 bbox는 bbox_cost_max + bbox_radius (flat zone),
 * 차선은 lane_cost_max + lane_radius (flat zone)로 차별 적용된다.
 *
 * bbox(PE 드럼, 직경 500mm)는 물리적 크기가 있어서 bbox_radius flat zone을 만들고,
 * 차선(10cm 흰색 테이프)은 lane_radius flat zone을 적용 (기본 0.0m).
 */
enum class PointType : uint8_t
{
  CONE = 0,  ///< LiDAR DBSCAN 결과 (PE 드럼) — flat zone 적용
  LANE = 1   ///< 카메라 차선 인식 결과 — 거리 기반 감쇠만 적용
};

/**
 * @brief 체이닝된 경계점 — 위치 + 원본 타입 정보
 *
 * DirectionChainer의 SideResult.component[]에 담긴 ChainPoint들이
 * to_chained_point()를 통해 변환된 후, CostmapGenerator에 전달된다.
 * ChainPoint보다 가벼운 구조체 — label, size 등 메타정보를 버리고
 * (x, y, type)만 남긴 것.
 */
struct ChainedPoint
{
  double x = 0.0;             ///< [m] base_link 기준 전방(+)/후방(-)
  double y = 0.0;             ///< [m] base_link 기준 좌측(+)/우측(-)
  PointType type = PointType::LANE;  ///< bbox/차선 구분 — costmap 비용 계산에 사용
  bool is_backbone = false;   ///< backbone 포인트 여부 — true이면 costmap에서 bbox_cost_max 적용

  /// Point2D로 변환 (costmap/planner 모듈과의 호환용)
  Point2D to_point2d() const { return {x, y}; }
};

/**
 * @brief DirectionChainer 입력 포인트 — 위치 + 메타정보
 *
 * BBox/LaneBoundary를 통합한 내부 표현.
 * ChainedPoint보다 label/size 정보가 추가됨.
 *
 * 데이터 흐름:
 *   LiDAR DBSCAN → BBox 메시지 → ChainPoint (type=CONE, label=cluster_id)
 *   카메라 차선  → LaneBoundary → ChainPoint (type=LANE, label=-1)
 */
struct ChainPoint
{
  double x = 0.0;             ///< [m] base_link 기준 전방(+)/후방(-)
  double y = 0.0;             ///< [m] base_link 기준 좌측(+)/우측(-)
  PointType type = PointType::LANE;  ///< bbox/차선 구분
  int32_t label = -1;         ///< 원본 cluster_id (bbox: DBSCAN 번호, 차선: -1)
  double size_x = 0.0;        ///< AABB X 크기 [m] (bbox만 유효)
  double size_y = 0.0;        ///< AABB Y 크기 [m] (bbox만 유효)
  bool is_backbone = false;   ///< backbone 포인트 여부 — true이면 costmap에서 bbox_cost_max 적용

  /// ChainedPoint로 변환 (CostmapGenerator 호환용, is_backbone 전파)
  ChainedPoint to_chained_point() const { return {x, y, type, is_backbone}; }
  /// Point2D로 변환 — 순수 좌표만 필요한 경우
  Point2D to_point2d() const { return {x, y}; }
};

}  // namespace chaining_costmap_ver

#endif  // CHAINING_COSTMAP_VER__COMMON__TYPES__POINT_TYPES_HPP_
